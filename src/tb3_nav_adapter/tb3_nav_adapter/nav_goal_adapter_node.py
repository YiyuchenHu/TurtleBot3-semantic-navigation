#!/usr/bin/env python3
"""
nav_goal_adapter_node.py — Stage-5 nav goal adapter ROS 2 node.

Subscribes to SemanticQueryResult from Stage 4, computes a safe approach
pose, optionally transforms it from base_link to map via TF, and publishes
a PoseStamped suitable for Nav2.

Subscribed topics
-----------------
  /semantic_query_node/selected_target   tb3_query/SemanticQueryResult

Published topics
----------------
  ~/goal_pose                            geometry_msgs/PoseStamped

Parameters  (see config/nav_goal_adapter.yaml)
----------
  approach_distance       float   Stand-off from target (m)
  min_standoff_distance   float   Reject targets closer than this (m)
  input_topic             str     SemanticQueryResult topic
  output_topic            str     PoseStamped goal topic
  target_frame            str     Desired output frame (default "map")
  tf_timeout              float   TF lookup timeout (s)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transform

from tb3_nav_adapter.goal_adapter_core import compute_approach_pose, yaw_to_quaternion

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None


class NavGoalAdapterNode(Node):

    def __init__(self) -> None:
        super().__init__("nav_goal_adapter_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query/msg/SemanticQueryResult not available. "
                "Did you source install/setup.bash after building tb3_query?"
            )
            raise RuntimeError("SemanticQueryResult message not found")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("approach_distance", 0.5)
        self.declare_parameter("min_standoff_distance", 0.3)
        self.declare_parameter("input_topic", "/semantic_query_node/selected_target")
        self.declare_parameter("output_topic", "~/goal_pose")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.5)

        self._approach_dist = self.get_parameter("approach_distance").value
        self._min_standoff  = self.get_parameter("min_standoff_distance").value
        in_topic            = self.get_parameter("input_topic").value
        out_topic           = self.get_parameter("output_topic").value
        self._target_frame  = self.get_parameter("target_frame").value
        self._tf_timeout    = self.get_parameter("tf_timeout").value

        # ── TF ────────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Sub / Pub ─────────────────────────────────────────────────────
        self.create_subscription(
            SemanticQueryResult, in_topic, self._query_cb, reliable_qos
        )
        self._goal_pub = self.create_publisher(PoseStamped, out_topic, reliable_qos)

        self.get_logger().info(
            "NavGoalAdapterNode ready  approach=%.2fm  standoff=%.2fm  target_frame=%s"
            % (self._approach_dist, self._min_standoff, self._target_frame)
        )

    # ── Callback ──────────────────────────────────────────────────────────

    def _query_cb(self, msg: SemanticQueryResult) -> None:
        if not msg.success:
            self.get_logger().debug(
                "Query unsuccessful, skipping: %s" % msg.status_message
            )
            return

        tx = msg.position.x
        ty = msg.position.y
        source_frame = msg.frame_id or "base_link"

        # compute_approach_pose expects the target RELATIVE TO THE ROBOT
        # (its contract: robot at the origin). The query node publishes
        # landmarks in its output_frame (map) without transforming them
        # into the robot frame, so feeding (tx, ty) straight in measured
        # the target's distance to the MAP ORIGIN, not to the robot: a
        # landmark near map (0,0) — e.g. the centre person of the default
        # world, whose map frame coincides with the Gazebo world frame
        # because the TB3 diff-drive plugin uses world odometry — was
        # always rejected as "too close", and every other approach pose
        # was laid along the origin→target ray instead of the
        # robot→target ray. Locate the robot in the message frame and
        # work with the relative vector; the returned goal is relative to
        # the robot, so shift it back before publishing.
        try:
            t = self._tf_buffer.lookup_transform(
                source_frame, "base_link", Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(
                "Cannot locate robot in %s (%s), skipping goal"
                % (source_frame, e)
            )
            return
        rx = t.transform.translation.x
        ry = t.transform.translation.y

        result = compute_approach_pose(
            tx - rx, ty - ry, self._approach_dist, self._min_standoff
        )

        if result is None:
            self.get_logger().warn(
                "Target %s too close to the robot (%.2fm), skipping goal"
                % (msg.object_id, ((tx - rx) ** 2 + (ty - ry) ** 2) ** 0.5)
            )
            return

        gx_rel, gy_rel, yaw = result
        gx, gy = rx + gx_rel, ry + gy_rel
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = source_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        # ── TF transform to target_frame ──────────────────────────────
        if self._target_frame != source_frame:
            try:
                goal = self._tf_buffer.transform(
                    goal,
                    self._target_frame,
                    timeout=Duration(seconds=self._tf_timeout),
                )
                self.get_logger().info(
                    "[%s] %s → approach (%.2f, %.2f) in %s  yaw=%.1f°"
                    % (
                        msg.semantic_name,
                        msg.object_id,
                        goal.pose.position.x,
                        goal.pose.position.y,
                        self._target_frame,
                        yaw * 57.2958,
                    )
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(
                    "TF %s→%s unavailable (%s), publishing in %s"
                    % (source_frame, self._target_frame, e, source_frame)
                )
        else:
            self.get_logger().info(
                "[%s] %s → approach (%.2f, %.2f) in %s  yaw=%.1f°"
                % (
                    msg.semantic_name,
                    msg.object_id,
                    gx, gy,
                    source_frame,
                    yaw * 57.2958,
                )
            )

        self._goal_pub.publish(goal)

        # TODO: optionally call NavigateToPose action client here


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
