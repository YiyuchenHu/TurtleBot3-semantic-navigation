#!/usr/bin/env python3
"""
semantic_memory_node.py — Stage-3 semantic memory ROS 2 node.

Subscribes to localized 3-D detections from tb3_localizer, maintains a
persistent in-memory registry of unique semantic objects, and periodically
publishes the current world state.

Subscribed topics
-----------------
  /localizer_node/localized_objects   vision_msgs/Detection3DArray

Published topics
----------------
  ~/objects                           vision_msgs/Detection3DArray
      One Detection3D per active remembered object.
      - results[0].hypothesis.class_id  = detector_label
      - results[0].hypothesis.score     = avg_confidence
      - bbox.center.position.x/y/z     = smoothed position
      - id                              = object_id  (e.g. "person_0")

Parameters  (see config/semantic_memory.yaml)
----------
  match_distance_threshold   float   Max distance (m) for same-object matching
  position_smoothing_alpha   float   EMA weight for position updates
  stale_timeout              float   Seconds until unseen object marked stale
  remove_timeout             float   Seconds until stale object removed
  publish_rate               float   Hz for periodic state publication
  input_topic                str     Localized-object input topic
  output_topic               str     Memory output topic
  output_frame               str     Frame for published objects
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from tb3_memory.memory_core import MemoryCore, Observation


class SemanticMemoryNode(Node):

    def __init__(self) -> None:
        super().__init__("semantic_memory_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("match_distance_threshold", 1.0)
        self.declare_parameter("position_smoothing_alpha", 0.3)
        self.declare_parameter("stale_timeout", 5.0)
        self.declare_parameter("remove_timeout", 30.0)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("input_topic", "/localizer_node/localized_objects")
        self.declare_parameter("output_topic", "~/objects")
        self.declare_parameter("output_frame", "base_link")

        match_dist = self.get_parameter("match_distance_threshold").value
        alpha      = self.get_parameter("position_smoothing_alpha").value
        stale_t    = self.get_parameter("stale_timeout").value
        remove_t   = self.get_parameter("remove_timeout").value
        pub_rate   = self.get_parameter("publish_rate").value
        in_topic   = self.get_parameter("input_topic").value
        out_topic  = self.get_parameter("output_topic").value
        self._out_frame = self.get_parameter("output_frame").value

        # ── Core registry ─────────────────────────────────────────────────
        self._core = MemoryCore(
            match_distance_threshold=match_dist,
            position_smoothing_alpha=alpha,
            stale_timeout=stale_t,
            remove_timeout=remove_t,
        )

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            Detection3DArray, in_topic, self._observation_cb, reliable_qos
        )

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(Detection3DArray, out_topic, reliable_qos)

        # ── Periodic publish + aging timer ────────────────────────────────
        period = 1.0 / max(pub_rate, 0.01)
        self._timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            "SemanticMemoryNode ready  "
            "match_dist=%.2fm  alpha=%.2f  stale=%.1fs  remove=%.1fs  rate=%.1fHz"
            % (match_dist, alpha, stale_t, remove_t, pub_rate)
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _observation_cb(self, msg: Detection3DArray) -> None:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id
            conf  = det.results[0].hypothesis.score
            x     = det.bbox.center.position.x
            y     = det.bbox.center.position.y

            obs = Observation(
                detector_label=label,
                confidence=conf,
                x=x,
                y=y,
                frame_id=msg.header.frame_id,
                timestamp=stamp_sec,
            )

            obj = self._core.update(obs)

            self.get_logger().info(
                "[%s] id=%s  pos=(%.2f, %.2f)  seen=%d  conf=%.2f  active=%s"
                % (
                    obj.detector_label,
                    obj.object_id,
                    obj.x,
                    obj.y,
                    obj.times_seen,
                    obj.avg_confidence,
                    obj.active,
                )
            )

    def _timer_cb(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._core.age(now_sec)

        active = self._core.get_active_objects()

        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._out_frame

        for obj in active:
            d3 = Detection3D()
            d3.header = out.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.detector_label
            hyp.hypothesis.score = float(obj.avg_confidence)
            d3.results.append(hyp)

            d3.bbox.center.position.x = obj.x
            d3.bbox.center.position.y = obj.y
            d3.bbox.center.position.z = 0.0

            d3.id = obj.object_id
            out.detections.append(d3)

        self._pub.publish(out)

        if active:
            self.get_logger().debug(
                "Published %d active objects (registry total: %d)"
                % (len(active), self._core.size)
            )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
