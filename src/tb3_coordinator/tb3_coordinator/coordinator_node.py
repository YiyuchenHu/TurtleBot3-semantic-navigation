#!/usr/bin/env python3
"""
coordinator_node.py — Lightweight mode manager for semantic navigation.

Manages transitions between frontier exploration and semantic-target
navigation.  When a user command arrives the coordinator:
  1. pauses exploration  (exploration_enabled = False)
  2. cancels any in-flight Nav2 goal
  3. forwards the command to semantic_query_node
  4. when a goal pose arrives from nav_goal_adapter, sends it to Nav2
  5. waits for Nav2 result (SUCCEEDED / ABORTED / CANCELED)
  6. resumes exploration after a configurable delay

State model
-----------
  EXPLORING ──(user cmd)──► SEMANTIC_QUERYING ──(query ok)──► SEMANTIC_NAV
      ▲                          │(query fail)                    │
      │                          ▼                                │ Nav2 result
      │                     TARGET_FAILED ◄── Nav2 fail ──────────┤
      │                          │                                ▼
      └──(resume delay)──────────┘                          TARGET_REACHED
                                                                 │
                                 └──(resume delay)───────────────┘

Subscribed topics
-----------------
  /user_command                          std_msgs/String
  /semantic_query_node/selected_target   tb3_query/SemanticQueryResult
  /nav_goal_adapter_node/goal_pose       geometry_msgs/PoseStamped

Published topics
----------------
  /exploration_enabled                   std_msgs/Bool
  /semantic_query_node/command           std_msgs/String
  ~/status                               std_msgs/String
"""

from __future__ import annotations

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

try:
    from tb3_query.msg import SemanticQueryResult
except ImportError:
    SemanticQueryResult = None


class Mode(Enum):
    IDLE = auto()
    EXPLORING = auto()
    SEMANTIC_QUERYING = auto()
    SEMANTIC_NAV = auto()
    TARGET_REACHED = auto()
    TARGET_FAILED = auto()
    PERCEPTION_SWEEP = auto()


class CoordinatorNode(Node):

    def __init__(self) -> None:
        super().__init__("coordinator_node")

        if SemanticQueryResult is None:
            self.get_logger().fatal(
                "tb3_query messages not found. Source install/setup.bash."
            )
            raise RuntimeError("SemanticQueryResult not available")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("user_command_topic", "/user_command")
        self.declare_parameter("exploration_enabled_topic", "/exploration_enabled")
        self.declare_parameter("query_command_topic", "/semantic_query_node/command")
        self.declare_parameter("query_result_topic", "/semantic_query_node/selected_target")
        self.declare_parameter("goal_pose_topic", "/nav_goal_adapter_node/goal_pose")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("auto_resume_exploration", True)
        self.declare_parameter("resume_delay_sec", 3.0)
        self.declare_parameter("nav_goal_timeout_sec", 60.0)
        self.declare_parameter("enable_perception_sweep", False)
        self.declare_parameter("sweep_angular_vel", 0.4)
        self.declare_parameter("sweep_duration_sec", 18.0)

        user_cmd_topic   = self.get_parameter("user_command_topic").value
        expl_en_topic    = self.get_parameter("exploration_enabled_topic").value
        query_cmd_topic  = self.get_parameter("query_command_topic").value
        query_res_topic  = self.get_parameter("query_result_topic").value
        goal_pose_topic  = self.get_parameter("goal_pose_topic").value
        status_topic     = self.get_parameter("status_topic").value
        nav_action_name  = self.get_parameter("navigate_to_pose_action").value
        self._auto_resume = self.get_parameter("auto_resume_exploration").value
        self._resume_delay = self.get_parameter("resume_delay_sec").value
        self._nav_timeout = self.get_parameter("nav_goal_timeout_sec").value
        self._sweep_enabled = self.get_parameter("enable_perception_sweep").value
        self._sweep_ang_vel = self.get_parameter("sweep_angular_vel").value
        self._sweep_duration = self.get_parameter("sweep_duration_sec").value

        # ── State ─────────────────────────────────────────────────────────
        self._mode = Mode.EXPLORING
        self._resume_timer = None
        self._nav_goal_handle = None
        self._nav_timeout_timer = None
        self._sweep_timer = None
        self._sweep_done = False

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._expl_en_pub = self.create_publisher(Bool, expl_en_topic, reliable_qos)
        self._query_cmd_pub = self.create_publisher(String, query_cmd_topic, reliable_qos)
        self._status_pub = self.create_publisher(String, status_topic, reliable_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", reliable_qos)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            String, user_cmd_topic, self._user_command_cb, reliable_qos)
        self.create_subscription(
            SemanticQueryResult, query_res_topic, self._query_result_cb, reliable_qos)
        self.create_subscription(
            PoseStamped, goal_pose_topic, self._goal_pose_cb, reliable_qos)

        # ── Nav2 action client ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, nav_action_name)

        self._set_exploration(True)
        self._publish_status("started in EXPLORING mode")

        self.create_timer(5.0, self._check_sweep_trigger)
        self.get_logger().info(
            "CoordinatorNode ready — mode=EXPLORING  nav_action=%s" % nav_action_name)

    # ── Mode helpers ──────────────────────────────────────────────────────

    def _set_mode(self, mode: Mode) -> None:
        old = self._mode
        self._mode = mode
        self._publish_status("%s → %s" % (old.name, mode.name))
        self.get_logger().info("[mode] %s → %s" % (old.name, mode.name))

    def _set_exploration(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._expl_en_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = "[%s] %s" % (self._mode.name, text)
        self._status_pub.publish(msg)

    # ── Cancel any active Nav2 goal ───────────────────────────────────────

    def _cancel_nav_goal(self) -> None:
        if self._nav_timeout_timer is not None:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None

        if self._nav_goal_handle is not None:
            self.get_logger().info("[nav2] Canceling current goal")
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _user_command_cb(self, msg: String) -> None:
        cmd = msg.data.strip()
        if not cmd:
            return

        self.get_logger().info("User command: '%s'" % cmd)

        if self._resume_timer is not None:
            self._resume_timer.cancel()
            self._resume_timer = None

        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self._sweep_timer = None
            cmd = Twist()
            self._cmd_vel_pub.publish(cmd)
        self._cancel_nav_goal()
        self._set_exploration(False)
        self._set_mode(Mode.SEMANTIC_QUERYING)

        fwd = String()
        fwd.data = cmd
        self._query_cmd_pub.publish(fwd)
        self._publish_status("forwarded command: '%s'" % cmd)

    def _query_result_cb(self, msg: SemanticQueryResult) -> None:
        if self._mode != Mode.SEMANTIC_QUERYING:
            return

        if msg.success:
            self._set_mode(Mode.SEMANTIC_NAV)
            self._publish_status(
                "target selected: %s (%s) at (%.2f, %.2f) — waiting for goal pose"
                % (msg.object_id, msg.semantic_name,
                   msg.position.x, msg.position.y)
            )
        else:
            self.get_logger().warn("Query failed: %s" % msg.status_message)
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("query failed: %s" % msg.status_message)
            self._schedule_resume()

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        if self._mode != Mode.SEMANTIC_NAV:
            return

        self._publish_status(
            "goal pose received: (%.2f, %.2f) in %s — sending to Nav2"
            % (msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
        )
        self._send_nav2_goal(msg)

    # ── Nav2 action execution ─────────────────────────────────────────────

    def _send_nav2_goal(self, pose: PoseStamped) -> None:
        if not self._nav_client.server_is_ready():
            self.get_logger().warn("[nav2] Action server not ready, failing")
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("Nav2 action server not available")
            self._schedule_resume()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            "[nav2] Sending goal (%.2f, %.2f) in %s"
            % (pose.pose.position.x, pose.pose.position.y, pose.header.frame_id)
        )

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[nav2] Goal REJECTED")
            self._nav_goal_handle = None
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("Nav2 goal rejected")
            self._schedule_resume()
            return

        self.get_logger().info("[nav2] Goal ACCEPTED — navigating")
        self._nav_goal_handle = goal_handle
        self._publish_status("Nav2 goal accepted — robot is moving")

        if self._nav_timeout > 0:
            self._nav_timeout_timer = self.create_timer(
                self._nav_timeout, self._nav_timeout_cb)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg) -> None:
        pass

    def _nav_timeout_cb(self) -> None:
        if self._nav_timeout_timer is not None:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None

        if self._mode == Mode.SEMANTIC_NAV and self._nav_goal_handle is not None:
            self.get_logger().warn(
                "[nav2] Goal timed out after %.1fs, canceling" % self._nav_timeout)
            self._cancel_nav_goal()
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("Nav2 goal timed out")
            self._schedule_resume()

    def _nav_result_cb(self, future) -> None:
        if self._nav_timeout_timer is not None:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None

        self._nav_goal_handle = None

        if self._mode != Mode.SEMANTIC_NAV:
            return

        result = future.result()
        status = result.status

        # action_msgs/GoalStatus constants
        STATUS_SUCCEEDED = 4
        STATUS_ABORTED = 6
        STATUS_CANCELED = 5

        if status == STATUS_SUCCEEDED:
            self.get_logger().info("[nav2] Goal SUCCEEDED")
            self._set_mode(Mode.TARGET_REACHED)
            self._publish_status("Nav2 goal succeeded — target reached")
        elif status == STATUS_CANCELED:
            self.get_logger().warn("[nav2] Goal CANCELED")
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("Nav2 goal canceled")
        else:
            self.get_logger().warn("[nav2] Goal FAILED (status=%d)" % status)
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("Nav2 goal failed (status=%d)" % status)

        self._schedule_resume()

    # ── Perception sweep ──────────────────────────────────────────────

    def _check_sweep_trigger(self) -> None:
        """Trigger a 360 perception sweep once after exploration completes.

        When exploration is active but the robot hasn't moved for several checks
        (no frontiers left), start a sweep to accumulate semantic observations.
        """
        if not self._sweep_enabled or self._sweep_done:
            return
        if self._mode != Mode.EXPLORING:
            self._idle_explore_count = 0
            return
        if self._nav_goal_handle is not None:
            self._idle_explore_count = 0
            return
        self._idle_explore_count = getattr(self, "_idle_explore_count", 0) + 1
        if self._idle_explore_count >= 6:
            self._start_sweep()

    def _start_sweep(self) -> None:
        self.get_logger().info(
            "[sweep] Starting 360° perception sweep (%.1fs at %.2f rad/s)"
            % (self._sweep_duration, self._sweep_ang_vel))
        self._set_exploration(False)
        self._set_mode(Mode.PERCEPTION_SWEEP)
        self._sweep_timer = self.create_timer(0.1, self._sweep_tick)
        self._sweep_start = self.get_clock().now()

    def _sweep_tick(self) -> None:
        elapsed = (self.get_clock().now() - self._sweep_start).nanoseconds / 1e9
        if elapsed >= self._sweep_duration:
            self._stop_sweep()
            return
        cmd = Twist()
        cmd.angular.z = self._sweep_ang_vel
        self._cmd_vel_pub.publish(cmd)

    def _stop_sweep(self) -> None:
        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self._sweep_timer = None
        cmd = Twist()
        self._cmd_vel_pub.publish(cmd)
        self._sweep_done = True
        self.get_logger().info("[sweep] Perception sweep complete")
        self._set_exploration(True)
        self._set_mode(Mode.EXPLORING)

    # ── Resume exploration ────────────────────────────────────────────────

    def _schedule_resume(self) -> None:
        if not self._auto_resume:
            self.get_logger().info(
                "Auto-resume disabled, staying in %s" % self._mode.name)
            return

        self.get_logger().info(
            "Will resume exploration in %.1fs" % self._resume_delay)

        if self._resume_timer is not None:
            self._resume_timer.cancel()

        self._resume_timer = self.create_timer(
            self._resume_delay, self._resume_exploration_cb)

    def _resume_exploration_cb(self) -> None:
        if self._resume_timer is not None:
            self._resume_timer.cancel()
            self._resume_timer = None

        self._set_exploration(True)
        self._set_mode(Mode.EXPLORING)
        self._publish_status("exploration resumed")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
