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
      │              (search suggested)│                          │ Nav2 result
      │                          ▼    ▼                           │
      │                SEARCHING   TARGET_FAILED ◄── Nav2 fail ───┤
      │                 │   │            ▲                        ▼
      │   (landmark appears) │(exploration                  TARGET_REACHED
      │                 │   │ complete: NOT FOUND)                │
      │                 ▼   └────────────┘                        │
      │        SEMANTIC_QUERYING (re-query, memory now hits)      │
      └──(resume delay, from TARGET_REACHED / TARGET_FAILED)──────┘

Search link (Phase 2c): a query whose class has no landmark yet comes
back with search_suggested. The coordinator then injects the expression
into the detect vocabulary (latched /active_search), re-enables
exploration, and watches the landmark topic for a promoted landmark of
that class (promotion threshold search_min_observations, enforced by
semantic_map_memory_node). Found → pause exploration and re-forward the
original command (memory now resolves it; the normal goal chain runs).
The injection stays active until the navigation reaches a terminal
state, then it is cleared — the landmark persists. Exploration complete
(latched /exploration_complete from goal_assignment_node) with no
landmark → explicit NOT FOUND.

Preemption (Phase 2c, R02b unresolved #3): a user command that arrives
while a Nav2 goal is in flight is queued, the goal is cancelled, and
the command is dispatched only when the old goal reports a terminal
state — its ABORTED/CANCELED result never bleeds into TARGET_FAILED
for the new command.

Subscribed topics
-----------------
  /user_command                             std_msgs/String
  /semantic_query_node/selected_target      tb3_query/SemanticQueryResult
  /nav_goal_adapter_node/goal_pose          geometry_msgs/PoseStamped
  /semantic_map_memory_node/landmark_objects vision_msgs/Detection3DArray
  /exploration_complete                     std_msgs/Bool (latched)
  /odom                                     nav_msgs/Odometry (search mileage)

Published topics
----------------
  /exploration_enabled                   std_msgs/Bool
  /semantic_query_node/command           std_msgs/String
  /active_search                         std_msgs/String (latched; ''=none)
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
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray
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
    SEARCHING = auto()
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
        # R04 demo shape: after a navigation terminal state (TARGET_REACHED
        # or TARGET NOT FOUND) go to IDLE and hold position instead of
        # auto-resuming exploration. Takes precedence over
        # auto_resume_exploration; set false to restore the old behavior.
        self.declare_parameter("hold_after_goal", True)
        self.declare_parameter("resume_delay_sec", 3.0)
        self.declare_parameter("nav_goal_timeout_sec", 60.0)
        self.declare_parameter("enable_perception_sweep", True)
        self.declare_parameter("sweep_angular_vel", 0.4)
        self.declare_parameter("sweep_duration_sec", 18.0)
        # ── Search link (Phase 2c) ────────────────────────────────────
        self.declare_parameter("active_search_topic", "/active_search")
        self.declare_parameter("landmark_topic",
                               "/semantic_map_memory_node/landmark_objects")
        self.declare_parameter("exploration_complete_topic",
                               "/exploration_complete")
        self.declare_parameter("odom_topic", "/odom")
        # Grace before an already-latched "exploration complete" is
        # honored as NOT FOUND: gives the detector a chance to see the
        # target from the current pose when the map was already fully
        # explored before the search began.
        self.declare_parameter("search_grace_sec", 15.0)
        # ── Search patrol (R05) ──────────────────────────────────────
        # Frontier exploration finishes when the MAP is complete, which
        # is not the same as having LOOKED at everything: the frontier
        # goals always face outward, so objects in the interior are only
        # ever seen in passing. Measured (R05 A1/A2): once exploration
        # completed (~400 s), every later search died at the grace
        # window with the robot standing still — the green-shirt
        # candidate sat at n=4/5, 0.05 m from the true person, and
        # expired. Instead of giving up, patrol: drive to a point the
        # search has not covered and turn on the spot there (the sweep
        # speed is below the detector's spin gate, so frames still
        # count). Each round = goal + sweep; NOT FOUND only after the
        # budget is spent.
        self.declare_parameter("search_patrol_enabled", True)
        self.declare_parameter("search_patrol_rounds", 4)
        self.declare_parameter("search_patrol_sweep_sec", 14.0)
        self.declare_parameter("search_patrol_goal_timeout_sec", 60.0)
        # Patrol waypoints in the map frame, "x:y". Default = the four
        # interior quadrant centres of the 6x6 m room: close enough to
        # every object that a sweep sees it inside the memory range gate
        # (2.5 m), far enough from the walls to be reachable.
        self.declare_parameter("search_patrol_points",
                               ["1.2:1.2", "-1.2:1.2", "-1.2:-1.2", "1.2:-1.2"])
        # Watchdog for the preemption path: if the cancelled goal's
        # result callback never arrives, dispatch the queued command
        # anyway after this long.
        self.declare_parameter("preempt_watchdog_sec", 5.0)

        user_cmd_topic   = self.get_parameter("user_command_topic").value
        expl_en_topic    = self.get_parameter("exploration_enabled_topic").value
        query_cmd_topic  = self.get_parameter("query_command_topic").value
        query_res_topic  = self.get_parameter("query_result_topic").value
        goal_pose_topic  = self.get_parameter("goal_pose_topic").value
        status_topic     = self.get_parameter("status_topic").value
        nav_action_name  = self.get_parameter("navigate_to_pose_action").value
        self._auto_resume = self.get_parameter("auto_resume_exploration").value
        self._hold_after_goal = self.get_parameter("hold_after_goal").value
        self._resume_delay = self.get_parameter("resume_delay_sec").value
        self._nav_timeout = self.get_parameter("nav_goal_timeout_sec").value
        self._sweep_enabled = self.get_parameter("enable_perception_sweep").value
        self._sweep_ang_vel = self.get_parameter("sweep_angular_vel").value
        self._sweep_duration = self.get_parameter("sweep_duration_sec").value
        self._search_grace = self.get_parameter("search_grace_sec").value
        self._preempt_watchdog_sec = self.get_parameter("preempt_watchdog_sec").value
        self._patrol_enabled = bool(
            self.get_parameter("search_patrol_enabled").value)
        self._patrol_rounds = int(
            self.get_parameter("search_patrol_rounds").value)
        self._patrol_sweep_sec = float(
            self.get_parameter("search_patrol_sweep_sec").value)
        self._patrol_goal_timeout = float(
            self.get_parameter("search_patrol_goal_timeout_sec").value)
        self._patrol_points = []
        for entry in self.get_parameter("search_patrol_points").value:
            sx, _, sy = str(entry).partition(":")
            if sx and sy:
                self._patrol_points.append((float(sx), float(sy)))

        # ── State ─────────────────────────────────────────────────────────
        self._mode = Mode.EXPLORING
        self._resume_timer = None
        self._nav_goal_handle = None
        self._nav_timeout_timer = None
        self._sweep_timer = None
        self._sweep_done = False
        # Search link
        self._search_label = None       # injected expression, or None
        self._search_query = None       # original user command to re-forward
        self._search_t0 = None          # rclpy Time at search start
        self._search_dist = 0.0         # odometry mileage during SEARCHING
        self._search_check_timer = None
        self._expl_complete = False     # latest latched /exploration_complete
        self._last_odom_xy = None
        # Search patrol
        self._patrol_active = False
        self._patrol_round = 0
        self._patrol_goal_handle = None
        self._patrol_timer = None
        self._patrol_sweep_t0 = None
        self._patrol_goal_t0 = None
        # Preemption
        self._pending_cmd = None
        self._preempt_timer = None
        self._last_cmd = None

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Latched QoS for the search-link topics: subscribers that start
        # later (or restart) still see the current state.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._expl_en_pub = self.create_publisher(Bool, expl_en_topic, reliable_qos)
        self._query_cmd_pub = self.create_publisher(String, query_cmd_topic, reliable_qos)
        self._status_pub = self.create_publisher(String, status_topic, reliable_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", reliable_qos)
        self._active_search_pub = self.create_publisher(
            String, self.get_parameter("active_search_topic").value, latched_qos)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            String, user_cmd_topic, self._user_command_cb, reliable_qos)
        self.create_subscription(
            SemanticQueryResult, query_res_topic, self._query_result_cb, reliable_qos)
        self.create_subscription(
            PoseStamped, goal_pose_topic, self._goal_pose_cb, reliable_qos)
        self.create_subscription(
            Detection3DArray, self.get_parameter("landmark_topic").value,
            self._landmark_cb, reliable_qos)
        self.create_subscription(
            Bool, self.get_parameter("exploration_complete_topic").value,
            self._expl_complete_cb, latched_qos)
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._odom_cb, sensor_qos)

        # ── Nav2 action client ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, nav_action_name)

        self._set_exploration(True)
        # Clean latched state: a subscriber that outlived a previous
        # coordinator (crash mid-search) drops its stale injection.
        self._publish_active_search("")
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
            # (Was `cmd = Twist()` — shadowed the command string and the
            # Twist went out on the query topic. Latent because the sweep
            # is disabled by default.)
            self._cmd_vel_pub.publish(Twist())

        # A search belonging to the PREVIOUS command dies here; the new
        # command decides for itself whether it needs one.
        if self._search_label is not None:
            self._clear_search("preempted by new command")

        # ── Preemption (R02b unresolved #3) ──────────────────────────────
        # Never race a new goal against an in-flight one: queue the
        # command, cancel the goal, and dispatch when its terminal result
        # arrives (_nav_result_cb) or the watchdog fires.
        if self._nav_goal_handle is not None:
            self._pending_cmd = cmd
            self._publish_status(
                "preempting in-flight goal, command queued: '%s'" % cmd)
            self._cancel_nav_goal()
            if self._preempt_timer is not None:
                self._preempt_timer.cancel()
            self._preempt_timer = self.create_timer(
                self._preempt_watchdog_sec, self._preempt_watchdog)
            return

        self._dispatch_command(cmd)

    def _dispatch_command(self, cmd: str) -> None:
        self._set_exploration(False)
        self._set_mode(Mode.SEMANTIC_QUERYING)
        self._last_cmd = cmd

        fwd = String()
        fwd.data = cmd
        self._query_cmd_pub.publish(fwd)
        self._publish_status("forwarded command: '%s'" % cmd)

    def _preempt_watchdog(self) -> None:
        if self._preempt_timer is not None:
            self._preempt_timer.cancel()
            self._preempt_timer = None
        if self._pending_cmd is not None:
            cmd = self._pending_cmd
            self._pending_cmd = None
            self.get_logger().warn(
                "[preempt] cancelled goal never reported terminal state — "
                "dispatching queued command '%s' anyway" % cmd)
            self._dispatch_command(cmd)

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
        elif msg.search_suggested and msg.search_label:
            self._begin_search(msg.search_label, msg.query_text)
        else:
            self.get_logger().warn("Query failed: %s" % msg.status_message)
            self._set_mode(Mode.TARGET_FAILED)
            self._publish_status("query failed: %s" % msg.status_message)
            self._schedule_resume()

    # ── Search link (Phase 2c) ────────────────────────────────────────────

    def _publish_active_search(self, label: str) -> None:
        msg = String()
        msg.data = label
        self._active_search_pub.publish(msg)

    def _begin_search(self, label: str, query_text: str) -> None:
        self._search_label = label
        self._search_query = query_text or self._last_cmd
        self._search_t0 = self.get_clock().now()
        self._search_dist = 0.0
        self._abort_patrol()   # fresh budget per search
        self._publish_active_search(label)
        self._set_mode(Mode.SEARCHING)
        self._set_exploration(True)
        self.get_logger().info(
            "[search] '%s' not in memory — injected into detect vocabulary, "
            "exploration enabled (grace %.0fs, expl_complete=%s)"
            % (label, self._search_grace, self._expl_complete))
        self._publish_status(
            "SEARCHING for '%s' — vocabulary injected, exploring" % label)
        if self._search_check_timer is not None:
            self._search_check_timer.cancel()
        self._search_check_timer = self.create_timer(
            5.0, self._search_check_tick)

    def _clear_search(self, reason: str) -> None:
        self._abort_patrol()
        if self._search_label is None:
            return
        label = self._search_label
        self._search_label = None
        self._search_query = None
        if self._search_check_timer is not None:
            self._search_check_timer.cancel()
            self._search_check_timer = None
        self._publish_active_search("")
        elapsed = 0.0
        if self._search_t0 is not None:
            elapsed = (self.get_clock().now() - self._search_t0).nanoseconds / 1e9
        self.get_logger().info(
            "[search] cleared '%s' (%s) — vocabulary restored; landmark, "
            "if promoted, persists. search_elapsed=%.1fs mileage=%.2fm"
            % (label, reason, elapsed, self._search_dist))

    def _search_check_tick(self) -> None:
        """Periodic SEARCHING watchdog: once exploration is complete and
        the grace period has passed, patrol (if enabled and budget left),
        else declare NOT FOUND."""
        if self._mode != Mode.SEARCHING:
            return
        if self._patrol_active:
            return
        if not self._expl_complete:
            return
        elapsed = (self.get_clock().now() - self._search_t0).nanoseconds / 1e9
        if elapsed < self._search_grace:
            return
        if self._patrol_enabled and self._patrol_points \
                and self._patrol_round < self._patrol_rounds:
            self._start_patrol_round()
            return
        self._search_not_found()

    # ── Search patrol ────────────────────────────────────────────────────

    def _start_patrol_round(self) -> None:
        """Drive to the next patrol waypoint; sweep on arrival."""
        x, y = self._patrol_points[self._patrol_round % len(self._patrol_points)]
        self._patrol_round += 1
        self._patrol_active = True
        # Exploration off during patrol: its goals would fight ours, and
        # it has nothing left to offer anyway (that is why we are here).
        self._set_exploration(False)
        self.get_logger().info(
            "[patrol] round %d/%d for '%s' → waypoint (%.2f, %.2f)"
            % (self._patrol_round, self._patrol_rounds, self._search_label,
               x, y))
        self._publish_status(
            "search patrol %d/%d → (%.2f, %.2f)"
            % (self._patrol_round, self._patrol_rounds, x, y))
        if not self._nav_client.server_is_ready():
            self.get_logger().warn("[patrol] Nav2 not ready — sweeping here")
            self._begin_patrol_sweep()
            return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._patrol_goal_t0 = self.get_clock().now()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._patrol_goal_response_cb)

    def _patrol_goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("[patrol] waypoint goal rejected — sweeping here")
            self._begin_patrol_sweep()
            return
        self._patrol_goal_handle = handle
        handle.get_result_async().add_done_callback(self._patrol_goal_result_cb)

    def _patrol_goal_result_cb(self, _future) -> None:
        self._patrol_goal_handle = None
        # Arrived, aborted or cancelled — either way, look around from
        # wherever the robot ended up. A search that was already
        # answered mid-drive has left SEARCHING; do not sweep then.
        if self._mode == Mode.SEARCHING and self._patrol_active:
            self._begin_patrol_sweep()

    def _begin_patrol_sweep(self) -> None:
        self._patrol_sweep_t0 = self.get_clock().now()
        if self._patrol_timer is not None:
            self._patrol_timer.cancel()
        self._patrol_timer = self.create_timer(0.1, self._patrol_sweep_tick)
        self.get_logger().info(
            "[patrol] sweeping %.0fs at the waypoint" % self._patrol_sweep_sec)

    def _patrol_sweep_tick(self) -> None:
        if self._mode != Mode.SEARCHING:
            self._end_patrol_round()
            return
        elapsed = (self.get_clock().now()
                   - self._patrol_sweep_t0).nanoseconds / 1e9
        if elapsed >= self._patrol_sweep_sec:
            self._end_patrol_round()
            return
        cmd = Twist()
        cmd.angular.z = self._sweep_ang_vel
        self._cmd_vel_pub.publish(cmd)

    def _end_patrol_round(self) -> None:
        if self._patrol_timer is not None:
            self._patrol_timer.cancel()
            self._patrol_timer = None
        self._cmd_vel_pub.publish(Twist())
        self._patrol_active = False
        if self._mode != Mode.SEARCHING:
            return
        if self._patrol_round >= self._patrol_rounds:
            self.get_logger().info(
                "[patrol] budget spent (%d rounds) — declaring NOT FOUND"
                % self._patrol_round)
            self._search_not_found()
        # else: the next _search_check_tick starts the following round.

    def _abort_patrol(self) -> None:
        """Stop any patrol in flight (search ended, or a new command)."""
        if self._patrol_timer is not None:
            self._patrol_timer.cancel()
            self._patrol_timer = None
        if self._patrol_goal_handle is not None:
            self._patrol_goal_handle.cancel_goal_async()
            self._patrol_goal_handle = None
        if self._patrol_active:
            self._cmd_vel_pub.publish(Twist())
        self._patrol_active = False
        self._patrol_round = 0

    def _search_not_found(self) -> None:
        label = self._search_label
        elapsed = (self.get_clock().now() - self._search_t0).nanoseconds / 1e9
        mileage = self._search_dist
        self._clear_search("exploration complete, no landmark")
        self.get_logger().warn(
            "[search] TARGET NOT FOUND: '%s' — exploration complete, no "
            "landmark promoted (searched %.1fs, %.2fm)"
            % (label, elapsed, mileage))
        self._set_mode(Mode.TARGET_FAILED)
        self._publish_status(
            "TARGET NOT FOUND: '%s' — exploration complete, nothing promoted"
            % label)
        self._schedule_resume()

    def _landmark_cb(self, msg: Detection3DArray) -> None:
        if self._mode != Mode.SEARCHING or self._search_label is None:
            return
        hit = None
        for det in msg.detections:
            if det.results and \
                    det.results[0].hypothesis.class_id == self._search_label:
                hit = det
                break
        if hit is None:
            return
        elapsed = (self.get_clock().now() - self._search_t0).nanoseconds / 1e9
        self.get_logger().info(
            "[search] FOUND '%s' → %s at (%.2f, %.2f) n=%.0f after %.1fs, "
            "%.2fm explored — pausing exploration, re-running query"
            % (self._search_label, hit.id,
               hit.bbox.center.position.x, hit.bbox.center.position.y,
               hit.results[0].hypothesis.score, elapsed, self._search_dist))
        self._set_exploration(False)
        # Injection stays live until the navigation terminal state
        # (constraint: clear vocabulary after arrival) so the approach
        # keeps refining the landmark.
        query = self._search_query
        self._set_mode(Mode.SEMANTIC_QUERYING)
        fwd = String()
        fwd.data = query
        self._query_cmd_pub.publish(fwd)
        self._publish_status(
            "search found %s — re-forwarded query '%s'" % (hit.id, query))

    def _expl_complete_cb(self, msg: Bool) -> None:
        if msg.data == self._expl_complete:
            return
        self._expl_complete = msg.data
        self.get_logger().info(
            "[exploration] complete=%s (from goal_assignment)" % msg.data)
        # The periodic _search_check_tick handles SEARCHING: it applies
        # the grace window uniformly whether the signal was latched
        # before the search or arrived during it.

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_xy is not None and self._mode == Mode.SEARCHING:
            dx = x - self._last_odom_xy[0]
            dy = y - self._last_odom_xy[1]
            self._search_dist += (dx * dx + dy * dy) ** 0.5
        self._last_odom_xy = (x, y)

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

        # ── Preemption hand-off ──────────────────────────────────────────
        # A queued command waits for exactly this: the cancelled goal's
        # terminal result. Dispatch it now; the old goal's ABORTED /
        # CANCELED status must not turn into TARGET_FAILED.
        if self._pending_cmd is not None:
            if self._preempt_timer is not None:
                self._preempt_timer.cancel()
                self._preempt_timer = None
            cmd = self._pending_cmd
            self._pending_cmd = None
            try:
                status = future.result().status
            except Exception:  # noqa: BLE001 — dispatch regardless
                status = -1
            self.get_logger().info(
                "[preempt] old goal terminal (status=%d) — dispatching "
                "queued command '%s'" % (status, cmd))
            self._dispatch_command(cmd)
            return

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
        # Terminal-state choke point: every REACHED/FAILED path funnels
        # through here, so this is where a still-active search injection
        # is cleared (constraint: vocabulary cleared after arrival; the
        # landmark persists).
        if self._search_label is not None and self._mode in (
                Mode.TARGET_REACHED, Mode.TARGET_FAILED):
            self._clear_search("navigation terminal: %s" % self._mode.name)

        if self._hold_after_goal:
            terminal = self._mode.name
            self._set_exploration(False)
            self._cmd_vel_pub.publish(Twist())
            self._set_mode(Mode.IDLE)
            self.get_logger().info(
                "[hold] %s → IDLE — holding position, exploration stays off "
                "(hold_after_goal:=false restores auto-resume)" % terminal)
            self._publish_status(
                "holding position after %s — send a new command" % terminal)
            return

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
