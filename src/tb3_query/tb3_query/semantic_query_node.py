#!/usr/bin/env python3
"""
semantic_query_node.py — Stage-4 semantic query ROS 2 node.

Resolves simple text commands against the current semantic memory state
and publishes a SemanticQueryResult with the selected target object.

Subscribed topics
-----------------
  /semantic_memory_node/objects     vision_msgs/Detection3DArray
  ~/command                         std_msgs/String

Published topics
----------------
  ~/selected_target                 tb3_query/SemanticQueryResult
  ~/query_status                    std_msgs/String   (human-readable debug)

Parameters  (see config/semantic_query.yaml)
----------
  memory_topic          str     Memory state topic
  command_topic         str     Input command topic
  output_topic          str     Selected target topic
  status_topic          str     Debug status topic
  semantic_targets_file str     Path to semantic_targets.yaml
  output_frame          str     Frame for output positions
"""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Point

from ament_index_python.packages import get_package_share_directory

from tb3_query.query_core import (
    MemoryObject,
    attribute_expression,
    canonicalize_expression,
    extract_expression,
    load_target_mapping,
    parse_command,
    select_target,
)

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None


class SemanticQueryNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_query_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query/msg/SemanticQueryResult not available. "
                "Did you source install/setup.bash after building?"
            )
            raise RuntimeError("SemanticQueryResult message not found")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("memory_topic", "/semantic_memory_node/objects")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("output_topic", "~/selected_target")
        self.declare_parameter("status_topic", "~/query_status")
        self.declare_parameter("semantic_targets_file", "")
        self.declare_parameter("output_frame", "base_link")
        # Attribute resolution: an expression that is not any
        # landmark's class still resolves when a landmark carries it as an
        # attribute with at least this many distinct detections.
        self.declare_parameter("attribute_min_count", 2)

        mem_topic    = self.get_parameter("memory_topic").value
        cmd_topic    = self.get_parameter("command_topic").value
        out_topic    = self.get_parameter("output_topic").value
        status_topic = self.get_parameter("status_topic").value
        targets_file = self.get_parameter("semantic_targets_file").value
        self._out_frame = self.get_parameter("output_frame").value

        # ── Load semantic mapping ─────────────────────────────────────────
        if not targets_file:
            pkg_share = get_package_share_directory("tb3_frontier_exploration")
            targets_file = os.path.join(pkg_share, "config", "semantic_targets.yaml")

        self._sem2det, self._det2sem = load_target_mapping(targets_file)
        self._known_targets = set(self._sem2det.keys())
        self.get_logger().info(
            "Loaded %d semantic targets: %s"
            % (len(self._known_targets), sorted(self._known_targets))
        )

        # ── State: latest memory snapshot ─────────────────────────────────
        self._memory_objects: list[MemoryObject] = []

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Detection3DArray, mem_topic, self._memory_cb, reliable_qos
        )
        self.create_subscription(
            String, cmd_topic, self._command_cb, reliable_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._result_pub = self.create_publisher(
            SemanticQueryResult, out_topic, reliable_qos
        )
        self._status_pub = self.create_publisher(
            String, status_topic, reliable_qos
        )

        self.get_logger().info("SemanticQueryNode ready  frame=%s" % self._out_frame)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _memory_cb(self, msg: Detection3DArray) -> None:
        objs: list[MemoryObject] = []
        for det in msg.detections:
            if not det.results:
                continue
            objs.append(MemoryObject(
                object_id=det.id,
                detector_label=det.results[0].hypothesis.class_id,
                x=det.bbox.center.position.x,
                y=det.bbox.center.position.y,
                confidence=det.results[0].hypothesis.score,
                # results[1:] = attribute evidence attached by the map
                # memory (label -> distinct detections)
                attributes={r.hypothesis.class_id: float(r.hypothesis.score)
                            for r in det.results[1:]},
            ))
        self._memory_objects = objs

    def _command_cb(self, msg: String) -> None:
        raw = msg.data.strip()
        self.get_logger().info("Command received: '%s'" % raw)

        parsed = parse_command(raw, self._known_targets)

        if parsed is None:
            # ── Open-vocabulary path (search link) ───────────────────────
            # Not a registry target. If the text still reads as a
            # navigation command, the remainder becomes a free-form
            # expression: resolve it against memory by literal label
            # (search-promoted landmarks carry the expression as their
            # class), else suggest a vocabulary-injection search.
            expression = extract_expression(raw)
            if expression is None:
                self._publish_failure(
                    raw, "", "",
                    f"unsupported semantic target in: '{raw}' "
                    f"(known: {sorted(self._known_targets)})"
                )
                return
            canon = canonicalize_expression(expression)
            if canon != expression:
                self.get_logger().info(
                    "expression '%s' → '%s' (wording the detector grounds)"
                    % (expression, canon))
                expression = canon
            semantic_name = expression
            detector_label = expression
            desired_index = None
            self.get_logger().info(
                "Open-vocabulary expression: '%s'" % expression)
        else:
            semantic_name = parsed.semantic_name
            detector_label = self._sem2det.get(semantic_name, "")
            if not detector_label:
                self._publish_failure(
                    raw, semantic_name, "",
                    f"no detector_label mapping for '{semantic_name}'"
                )
                return
            desired_index = parsed.desired_index
            # ── Attribute-qualified class ────────────────────────────────
            # "find the person wearing a green shirt" token-matches the
            # registry class `person`, but injecting plain "person" would
            # stop the search at ANY person. Commands with attribute
            # tokens beyond the class reference demote to the
            # open-vocabulary expression path (full expression becomes
            # the injected vocabulary word and the landmark class).
            attr_expr = attribute_expression(raw, semantic_name,
                                             detector_label)
            if attr_expr is not None and desired_index is None:
                canon = canonicalize_expression(attr_expr)
                if canon != attr_expr:
                    self.get_logger().info(
                        "expression '%s' → '%s' (wording the detector grounds)"
                        % (attr_expr, canon))
                    attr_expr = canon
                semantic_name = attr_expr
                detector_label = attr_expr
                self.get_logger().info(
                    "Attribute-qualified '%s' → open-vocabulary "
                    "expression: '%s'" % (parsed.semantic_name, attr_expr))
            if desired_index is not None:
                self.get_logger().info(
                    "Parsed: semantic='%s'  desired_index=%d"
                    % (semantic_name, desired_index)
                )

        result = select_target(
            self._memory_objects,
            semantic_name,
            detector_label,
            raw,
            desired_index=desired_index,
            attribute_min_count=int(self.get_parameter("attribute_min_count").value),
        )

        # ── Search suggestion ────────────────────────────────────────────
        # The class has NO landmark at all and no specific index was
        # requested → the coordinator can inject `detector_label` into the
        # detect vocabulary and explore. With an index ("go to person 3")
        # a search could not resolve WHICH instance, so it stays a plain
        # failure. When landmarks exist, select_target already resolved
        # per memory (nearest / by index) and no search is suggested.
        search_suggested = (
            not result.success
            and desired_index is None
            and not any(o.detector_label == detector_label
                        for o in self._memory_objects)
        )

        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame
        out.success = result.success
        out.query_text = result.query_text
        out.semantic_name = result.semantic_name
        out.detector_label = result.detector_label
        out.object_id = result.object_id
        out.position = Point(x=result.x, y=result.y, z=0.0)
        out.frame_id = self._out_frame
        out.confidence = float(result.confidence)
        out.status_message = result.status_message
        out.search_suggested = search_suggested
        out.search_label = detector_label if search_suggested else ""

        self._result_pub.publish(out)

        status = String()
        status.data = result.status_message
        self._status_pub.publish(status)

        if result.success:
            self.get_logger().info(
                "Query OK: %s → %s at (%.2f, %.2f)"
                % (semantic_name, result.object_id, result.x, result.y)
            )
        elif search_suggested:
            self.get_logger().info(
                "Query MISS with search suggestion: inject '%s' "
                "into the detect vocabulary" % detector_label)
        else:
            self.get_logger().warn("Query FAILED: %s" % result.status_message)

    def _publish_failure(
        self, raw: str, semantic_name: str, detector_label: str, msg: str
    ) -> None:
        out = SemanticQueryResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame
        out.success = False
        out.query_text = raw
        out.semantic_name = semantic_name
        out.detector_label = detector_label
        out.status_message = msg
        self._result_pub.publish(out)

        status = String()
        status.data = msg
        self._status_pub.publish(status)
        self.get_logger().warn("Query FAILED: %s" % msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
