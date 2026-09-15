#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.hpp>

#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <vector>

/**
 * @file goal_assignment_node.cpp
 * @brief ROS 2 node: assign Nav2 navigation goals from frontier centroids for autonomous exploration.
 *
 * This node is the bridge between **candidate frontier generation** (frontier_detection_node) and **actual
 * autonomous navigation** (Nav2):
 *
 * - Subscribes to the frontier centroid topic (default `/frontiers`) as `geometry_msgs/PoseArray`, produced by
 *   the frontier detection node.
 * - Obtains the robot pose in the **map** frame using **TF** (`frame_id` → `robot_base_frame`, e.g. map → base_link).
 *   Odometry is subscribed for topic compatibility but pose selection uses TF only in the current implementation.
 * - **Filters** frontier candidates: drops goals too close to the robot and optionally avoids a disk around the
 *   last failed/canceled/aborted goal.
 * - **Selects** the best valid frontier using a **nearest-to-robot** policy among remaining candidates.
 * - Sends **`navigate_to_pose`** goals to Nav2 **asynchronously** via `rclcpp_action`.
 * - Handles **goal timeout** (cancel if navigation stalls), **success** (clears failure memory), **cancellation**,
 *   **rejection**, and **abort** (records position for avoidance on the next selection).
 * - **Records failed goals** so `selectBestFrontier` can skip centroids near recent failures.
 *
 * Together with frontier_detection_node, this closes the exploration loop: map → frontiers → Nav2 → motion → updated map.
 */

namespace tb3_frontier_exploration
{

enum class State { IDLE, NAVIGATING };

class GoalAssignmentNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Construct the goal assignment node: parameters, TF, subscriptions, Nav2 action client, and control timer.
   *
   * @param none No constructor arguments; all behavior is configured via ROS parameters.
   * @return N/A (constructs the node in place).
   *
   * Pipeline role:
   * - Connects the exploration executive to `/frontiers` and Nav2’s `navigate_to_pose` action; without this setup,
   *   frontier centroids would never become motion commands.
   *
   * Implementation summary:
   * 1. Declares parameters for frontier and odom topic names, Nav2 action name, map/base frames, timeouts,
   *    selection distances, timer rate, and exploration-complete log throttling.
   * 2. Creates `tf2_ros::Buffer` and `TransformListener` for map-frame robot position.
   * 3. Subscribes to `PoseArray` frontiers and `Odometry` (odom callback is currently a no-op).
   * 4. Creates the `NavigateToPose` action client and a wall timer at `rate` Hz to run `timerCallback`.
   * 5. Logs startup configuration (topics, distances, timeout).
   *
   * Notes:
   * - Initial FSM state is `IDLE`; the timer drives both idle goal dispatch and navigating timeout checks.
   * - Action server must be available (Nav2 bt_navigator) before goals are sent.
   */
  GoalAssignmentNode()
  : Node("goal_assignment_node"),
    state_(State::IDLE)
  {
    declare_parameter<std::string>("frontiers_topic", "/frontiers");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter<double>("goal_timeout", 60.0);
    declare_parameter<double>("rate", 1.0);
    declare_parameter<double>("min_frontier_distance", 0.5);
    declare_parameter<double>("failed_goal_avoidance_radius", 1.0);
    // Failed goals are remembered as a TIME-LIMITED SET, not a single slot.
    // The single slot was the direct cause of the 40 s limit cycle: a frontier
    // goal failed and was recorded, the fallback goal then failed and
    // OVERWROTE that record, which re-armed the very frontier that had just
    // failed. See reports/chat_return_0827_wedge.md.
    declare_parameter<double>("failed_goal_ttl_sec", 90.0);
    declare_parameter<int>("failed_goal_max_entries", 64);
    declare_parameter<double>("exploration_complete_log_interval", 5.0);
    declare_parameter<bool>("require_startup_warmup", false);
    declare_parameter<std::string>("startup_warmup_complete_topic", "exploration_warmup_complete");
    declare_parameter<double>("startup_warmup_timeout_sec", 90.0);
    declare_parameter<bool>("enable_repeated_centroid_filter", true);
    declare_parameter<double>("repeated_centroid_reject_radius", 0.45);
    declare_parameter<int>("repeated_centroid_max_visits", 2);

    // ── Bootstrap-drive fallback parameters ───────────────────────────────
    // When exploration is enabled but no valid frontier passes the filters,
    // sample N points on a circle of radius R around the robot, query the
    // global costmap, and send the lowest-cost reachable one to Nav2 as a
    // regular navigate_to_pose goal. This replaces the old in-place 360°
    // sweep (which provided almost no SLAM benefit because the laser only
    // sees current line-of-sight geometry).
    declare_parameter<bool>("enable_fallback_drive", true);
    declare_parameter<std::string>("fallback_costmap_topic", "/global_costmap/costmap");
    declare_parameter<double>("fallback_radius_m", 1.5);
    declare_parameter<int>("fallback_num_samples", 16);
    // OccupancyGrid cost is 0–100; we reject candidates above this. 60 is
    // well below the inflation lethal threshold but above free space.
    declare_parameter<int>("fallback_max_cost", 60);
    // Minimum gap between consecutive fallback dispatches; avoids re-sending
    // an unreachable goal every timer tick while Nav2 is rejecting it.
    declare_parameter<double>("fallback_cooldown_sec", 4.0);

    // ── Exploration-complete detection ────────────────────────────────────
    // When frontier_detection has reported 0 safe centroids for K consecutive
    // messages AND the robot has already completed at least one navigation
    // goal (i.e. exploration actually started), declare the SLAM scan
    // "complete". This stops the fallback path from generating endless
    // "wander inside the known map" goals (the symptom the operator hit:
    // the robot bouncing between (1.00, 1.44) ↔ (0.94, -0.28) forever
    // because anti-loop's 0.45 m radius is smaller than the fallback radius).
    declare_parameter<bool>("enable_exploration_complete_detection", true);
    declare_parameter<int>("empty_frontier_streak_threshold", 3);
    // Stalemate guard: also declare exploration complete when the timer
    // callback fails to dispatch ANY goal for this many seconds in a row,
    // even though /frontiers is still publishing non-empty PoseArrays.
    // This catches the failure mode where every centroid is rejected by
    // selectBestFrontier filters (min_frontier_distance, anti-loop,
    // failed_goal_avoidance) and selectFallbackGoal also returns nullopt.
    // Without this, the node would log "No valid frontiers" forever
    // while the bot sits idle next to a corner.
    declare_parameter<double>("stalemate_idle_sec", 20.0);
    // Exploration-complete detection used to be gated on "the robot has
    // reached at least one goal". That gate is closed exactly when the node
    // is deadlocked from the start (visit_history_ empty), so it disabled the
    // escape hatch in the one case that needed it. It is replaced by a
    // minimum-uptime guard: refuse to declare completion until exploration has
    // been ENABLED for this long. Measured from the moment exploration became
    // enabled, not from node construction, so a slow startup warm-up cannot
    // eat the guard interval.
    declare_parameter<double>("min_exploration_time_sec", 60.0);
    // Policy when complete: "stop" = stay idle; "return_home" = drive back
    // to the pose recorded at first successful TF lookup.
    declare_parameter<std::string>("on_exploration_complete", "stop");

    std::string frontiers_topic = get_parameter("frontiers_topic").as_string();
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string action_name = get_parameter("navigate_to_pose_action").as_string();
    std::string fallback_costmap_topic =
      get_parameter("fallback_costmap_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    frontiers_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      frontiers_topic, 10, std::bind(&GoalAssignmentNode::frontiersCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&GoalAssignmentNode::odomCallback, this, std::placeholders::_1));

    // Global costmap is published with TRANSIENT_LOCAL durability by Nav2
    // (so late joiners receive the latest grid). Match that on the
    // subscription to get the first frame immediately on connect.
    auto costmap_qos = rclcpp::QoS(1).transient_local().reliable();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      fallback_costmap_topic, costmap_qos,
      std::bind(&GoalAssignmentNode::costmapCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);

    require_startup_warmup_ = get_parameter("require_startup_warmup").as_bool();
    startup_warmup_done_ = !require_startup_warmup_;
    warmup_watch_start_ = now();
    std::string warmup_topic = get_parameter("startup_warmup_complete_topic").as_string();
    warmup_sub_ = create_subscription<std_msgs::msg::Bool>(
      warmup_topic, rclcpp::QoS(10),
      std::bind(&GoalAssignmentNode::warmupCompleteCallback, this, std::placeholders::_1));

    exploration_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "exploration_enabled", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        exploration_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "[exploration] %s via /exploration_enabled",
          exploration_enabled_ ? "ENABLED" : "DISABLED");
      });

    double rate_hz = get_parameter("rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&GoalAssignmentNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "[goal_assignment] Started: frontiers=%s, action=%s, min_frontier_dist=%.2f, failed_avoid_radius=%.2f, goal_timeout=%.1fs",
      frontiers_topic.c_str(), action_name.c_str(),
      get_parameter("min_frontier_distance").as_double(),
      get_parameter("failed_goal_avoidance_radius").as_double(),
      get_parameter("goal_timeout").as_double());
    if (get_parameter("enable_fallback_drive").as_bool()) {
      RCLCPP_INFO(get_logger(),
        "[goal_assignment] Fallback drive ENABLED: costmap=%s, radius=%.2fm, samples=%d, max_cost=%d, cooldown=%.1fs",
        fallback_costmap_topic.c_str(),
        get_parameter("fallback_radius_m").as_double(),
        static_cast<int>(get_parameter("fallback_num_samples").as_int()),
        static_cast<int>(get_parameter("fallback_max_cost").as_int()),
        get_parameter("fallback_cooldown_sec").as_double());
    }
    if (get_parameter("enable_exploration_complete_detection").as_bool()) {
      RCLCPP_INFO(get_logger(),
        "[goal_assignment] Exploration-complete detection ENABLED: streak_threshold=%d, stalemate=%.1fs, on_complete=%s",
        static_cast<int>(get_parameter("empty_frontier_streak_threshold").as_int()),
        get_parameter("stalemate_idle_sec").as_double(),
        get_parameter("on_exploration_complete").as_string().c_str());
    }
    if (require_startup_warmup_) {
      RCLCPP_INFO(get_logger(),
        "[goal_assignment] Startup warmup required; waiting for Bool true on '%s' (or timeout %.1fs)",
        warmup_topic.c_str(), get_parameter("startup_warmup_timeout_sec").as_double());
    }
  }

private:
  // Forward-declared early so member functions whose signatures use Point
  // (selectFallbackGoal, countSuccessfulVisitsNear, recordFailedGoal, etc.)
  // can reference the type. The actual storage members of type Point live
  // farther down with the rest of the data members.
  struct Point { double x, y; };

  void warmupCompleteCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      startup_warmup_done_ = true;
      RCLCPP_INFO(get_logger(), "[warmup] Received complete signal — frontier exploration enabled.");
    }
  }

  /**
   * @brief Store the latest frontier centroid message from frontier_detection_node under a mutex.
   *
   * @param msg Shared pointer to `geometry_msgs/PoseArray` (each pose’s position is a frontier centroid in map frame).
   * @return void; updates member `latest_frontiers_`.
   *
   * Pipeline role:
   * - Provides the **input queue** of exploration candidates for the timer-driven executive; without fresh messages,
   *   the node has nothing to navigate toward.
   *
   * Implementation summary:
   * 1. Lock `frontiers_mutex_`.
   * 2. Assign `latest_frontiers_ = msg` (shared pointer copy, replaces previous snapshot).
   * 3. Optionally log centroid count at DEBUG level.
   *
   * Notes:
   * - Each callback **replaces** the entire cached array when frontier_detection republishes on map update.
   * - Thread-safe with `timerCallback`, which reads the same pointer under the same mutex.
   */
  void frontiersCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      latest_frontiers_ = msg;
    }
    RCLCPP_DEBUG(get_logger(), "[frontiers] Received %zu centroids", msg->poses.size());

    // Update the streak counter used by exploration-complete detection.
    // frontier_detection publishes its safe centroids on this topic; an
    // empty array means "no safe frontier in the current map". After K
    // consecutive empty messages we assume the SLAM scan has converged.
    if (msg->poses.empty()) {
      ++empty_frontier_streak_;
    } else {
      empty_frontier_streak_ = 0;
      // If we previously declared the area "complete" and now the costmap
      // produced new frontiers (e.g. someone opened a door / pushed an
      // object), gracefully resume exploration. Also clear the stalemate
      // counter so a fresh resume does not immediately re-trigger
      // declaration on the next tick.
      if (exploration_complete_) {
        RCLCPP_INFO(get_logger(),
          "[exploration] New frontiers appeared (count=%zu) — resuming.",
          msg->poses.size());
        exploration_complete_ = false;
        idle_tick_streak_ = 0;
      }
    }
  }

  /**
   * @brief Odometry subscription callback (currently unused for pose or control).
   *
   * @param msg Latest `nav_msgs/Odometry` on `odom_topic` (ignored in the body).
   * @return void.
   *
   * Pipeline role:
   * - Reserved hook for future use (e.g. velocity checks or fallback pose); **does not** participate in frontier
   *   selection today—robot position comes from TF in `getRobotPoseInMap`.
   *
   * Implementation summary:
   * 1. Cast `msg` to void to silence unused-parameter warnings.
   *
   * Notes:
   * - Keeping the subscription allows the node to match launch files that expect an odom topic without extra nodes.
   * - Do not assume pose is updated here when reading or extending the code.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
  }

  /**
   * @brief Look up the robot base frame position projected into the map plane using TF.
   *
   * @param rx Output: robot x in `frame_id` (map) frame, meters.
   * @param ry Output: robot y in `frame_id` (map) frame, meters.
   * @return `true` if `lookupTransform(frame_id, robot_base_frame, ...)` succeeds; `false` on TF failure (outputs undefined).
   *
   * Pipeline role:
   * - Enables **distance-based** frontier filtering and **nearest-neighbor** selection; without a valid transform,
   *   the timer callback aborts before sending a goal.
   *
   * Implementation summary:
   * 1. Read `frame_id` and `robot_base_frame` from parameters.
   * 2. Call `tf_buffer_->lookupTransform` with latest time (`TimePointZero`) and 0.5 s timeout.
   * 3. On success, set `rx`, `ry` from `transform.translation` (x, y); on exception, log at DEBUG and return false.
   *
   * Notes:
   * - Only **translation** is used; yaw is not needed for Euclidean distance in the plane.
   * - Requires TF tree from `robot_base_frame` to `frame_id` to be published (e.g. map → odom → base_link).
   */
  bool getRobotPoseInMap(double & rx, double & ry)
  {
    std::string frame_id = get_parameter("frame_id").as_string();
    std::string robot_frame = get_parameter("robot_base_frame").as_string();
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        frame_id, robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
      // Memorize the very first successful pose as "home", used by the
      // return_home policy when exploration is complete. We use the first
      // valid TF rather than the launch-arg x_pose/y_pose because the
      // start frame coincides with map origin only after SLAM stabilizes.
      if (!start_pose_) {
        start_pose_ = Point{rx, ry};
        RCLCPP_INFO(get_logger(),
          "[start] Recorded start pose (%.2f, %.2f) — used as 'home' on exploration complete",
          rx, ry);
      }
      return true;
    } catch (const std::exception & e) {
      // Throttled to 1/5s so a stalled TF tree does not flood the log,
      // but the operator still sees the silent failure that previously
      // caused goal_assignment to look "frozen" (timer keeps running, no
      // [goal] Selected ever appears).
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[TF] Lookup %s -> %s failed (no goal will be sent until TF recovers): %s",
        frame_id.c_str(), robot_frame.c_str(), e.what());
      return false;
    }
  }

  /**
   * @brief Choose the nearest valid frontier index for the robot, applying distance and failure-avoidance filters.
   *
   * @param frontiers `PoseArray` of candidate centroids (positions in map frame; header may supply `frame_id`).
   * @param rx Robot x in map frame (meters), from TF.
   * @param ry Robot y in map frame (meters), from TF.
   * @param frame_id In/out: goal frame string; if empty on input, set from `frontiers.header.frame_id` when a candidate exists.
   * @return Index of the selected pose in `frontiers.poses`, or `std::nullopt` if every candidate is filtered out.
   *
   * Pipeline role:
   * - Implements the exploration **policy** (greedy nearest valid frontier); bridges perception output to a single
   *   actionable goal per timer cycle.
   *
   * Implementation summary:
   * 1. Load `min_frontier_distance` and `failed_goal_avoidance_radius` from parameters (squared for comparisons).
   * 2. For each frontier pose, compute squared distance to `(rx, ry)`; skip if closer than `min_frontier_distance`.
   * 3. Skip centroids within `failed_goal_avoidance_radius` of ANY unexpired entry in `failed_goals_`.
   * 4. Among remaining, track the index with minimum squared distance.
   * 5. If none remain, return nullopt; else if `frame_id` was empty, copy `frontiers.header.frame_id`, then return index.
   *
   * Notes:
   * - **Nearest** is Euclidean in the map **xy** plane; z is ignored.
   * - Ties are broken by **first** minimum encountered (order is array order).
   */
  std::optional<size_t> selectBestFrontier(
    const geometry_msgs::msg::PoseArray & frontiers,
    double rx, double ry,
    std::string & frame_id)
  {
    const double min_dist = get_parameter("min_frontier_distance").as_double();
    const double min_dist_sq = min_dist * min_dist;
    const double avoid_radius = get_parameter("failed_goal_avoidance_radius").as_double();
    const double avoid_radius_sq = avoid_radius * avoid_radius;

    size_t best = 0;
    double best_dist_sq = 1e30;
    bool found = false;

    for (size_t i = 0; i < frontiers.poses.size(); i++) {
      double gx = frontiers.poses[i].position.x;
      double gy = frontiers.poses[i].position.y;
      double dx = gx - rx;
      double dy = gy - ry;
      double d2 = dx * dx + dy * dy;

      if (d2 < min_dist_sq) {
        RCLCPP_DEBUG(get_logger(), "[select] Frontier [%zu] (%.2f, %.2f) skipped: too close (%.2f m)",
          i, gx, gy, std::sqrt(d2));
        continue;
      }
      if (isNearFailedGoal(gx, gy, avoid_radius_sq)) {
        RCLCPP_DEBUG(get_logger(), "[select] Frontier [%zu] (%.2f, %.2f) skipped: near a recent failed goal",
          i, gx, gy);
        continue;
      }

      if (get_parameter("enable_repeated_centroid_filter").as_bool()) {
        const double loop_r = get_parameter("repeated_centroid_reject_radius").as_double();
        const int max_vis = get_parameter("repeated_centroid_max_visits").as_int();
        if (loop_r > 0.0 && max_vis > 0) {
          const double loop_r_sq = loop_r * loop_r;
          const size_t n_vis = countSuccessfulVisitsNear(gx, gy, loop_r_sq);
          if (static_cast<int>(n_vis) >= max_vis) {
            RCLCPP_DEBUG(get_logger(),
              "[select] Frontier [%zu] (%.2f, %.2f) skipped: anti-loop (>= %d visits within %.2f m)",
              i, gx, gy, max_vis, loop_r);
            continue;
          }
        }
      }

      if (d2 < best_dist_sq) {
        best_dist_sq = d2;
        best = i;
        found = true;
      }
    }

    if (!found) return std::nullopt;
    if (frame_id.empty()) frame_id = frontiers.header.frame_id;
    return best;
  }

  /**
   * @brief Periodic executive: enforce Nav2 goal timeout when navigating; when idle, select and send the next frontier goal.
   *
   * @param none.
   * @return void; may call `async_cancel_goal` or `async_send_goal` as side effects.
   *
   * Pipeline role:
   * - **Central loop** for closed-loop exploration: supervises the current Nav2 goal and dispatches the next one
   *   when safe; without this timer, no goals would be issued after startup.
   *
   * Implementation summary:
   * 1. If state is `NAVIGATING` and a goal handle and send time exist: if elapsed time ≥ `goal_timeout`, record
   *    failure at `current_goal_x_/y_`, clear handle/time, set `IDLE`, call `async_cancel_goal`, return.
   * 2. If still `NAVIGATING` (no timeout), return (do not send another goal).
   * 3. If Nav2 action server not ready, return.
   * 4. Copy `latest_frontiers_` under mutex; if null or empty, call `logExplorationCompleteIfDue` and return.
   * 5. `getRobotPoseInMap`; on failure return.
   * 6. `selectBestFrontier`; if none, log and `logExplorationCompleteIfDue`, return.
   * 7. Log selection; erase chosen pose from `latest_frontiers_->poses` under mutex (avoid immediate duplicate from cache).
   * 8. Build `NavigateToPose::Goal` with identity quaternion; set `current_goal_x_/y_`; set state `NAVIGATING`;
   *    register `goal_response_callback` and `result_callback`; call `async_send_goal`.
   *
   * Notes:
   * - State is set to `NAVIGATING` **before** acceptance; if the server **rejects**, the response callback resets to `IDLE`.
   * - Timeout uses time **after** acceptance (`goal_sent_time_` set in response callback), not wall time from send call.
   * - Cancel path copies handle before reset so `async_cancel_goal` still receives a valid handle.
   */
  void timerCallback()
  {
    const double goal_timeout = get_parameter("goal_timeout").as_double();

    if (state_ == State::NAVIGATING) {
      if (current_goal_handle_ && goal_sent_time_) {
        double elapsed = (now() - *goal_sent_time_).seconds();
        if (elapsed >= goal_timeout) {
          RCLCPP_WARN(get_logger(), "[timeout] Goal stalled for %.1fs (limit %.1fs), canceling",
            elapsed, goal_timeout);
          auto handle = current_goal_handle_;
          current_goal_handle_.reset();
          goal_sent_time_.reset();
          recordFailedGoal(current_goal_x_, current_goal_y_);
          state_ = State::IDLE;
          if (handle) {
            nav_client_->async_cancel_goal(handle);
          }
        }
      }
      return;
    }

    if (!exploration_enabled_) {
      return;
    }

    if (require_startup_warmup_ && !startup_warmup_done_) {
      const double tout = get_parameter("startup_warmup_timeout_sec").as_double();
      if ((now() - warmup_watch_start_).seconds() >= tout) {
        if (!warmup_timeout_announced_) {
          RCLCPP_WARN(get_logger(),
            "[warmup] No complete signal after %.1fs — enabling exploration anyway.", tout);
          warmup_timeout_announced_ = true;
        }
        startup_warmup_done_ = true;
      } else {
        if (!last_warmup_waiting_log_ ||
          (now() - *last_warmup_waiting_log_).seconds() >= 5.0)
        {
          RCLCPP_INFO(get_logger(), "[warmup] Waiting for startup map warmup…");
          last_warmup_waiting_log_ = now();
        }
        return;
      }
    }

    if (!nav_client_->action_server_is_ready()) {
      // Throttled WARN (was DEBUG): if Nav2 lifecycle never finishes
      // bringup or bt_navigator deactivates, goal_assignment would silently
      // skip every cycle. Make this visible so the operator can correlate
      // with lifecycle_manager logs.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[idle] Nav2 navigate_to_pose action server not ready — no goal will be sent");
      return;
    }

    // Exploration is genuinely running from here on: enabled, warm-up done,
    // Nav2 answering. This is the zero point for min_exploration_time_sec.
    if (!exploration_start_time_) {
      exploration_start_time_ = now();
    }

    double rx, ry;
    if (!getRobotPoseInMap(rx, ry)) return;

    std::string frame_id = get_parameter("frame_id").as_string();

    // ── Exploration-complete short-circuit ───────────────────────────────
    // Once we have declared the SLAM scan complete, the only thing that
    // re-arms us is a non-empty `/frontiers` message (handled in
    // frontiersCallback). Skip both frontier and fallback paths so the
    // robot stops wandering inside a fully-scanned map.
    if (exploration_complete_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[complete] Idle — scan finished. Will resume if new frontiers appear.");
      return;
    }

    // ── Path 1: prefer a real frontier centroid ─────────────────────────
    geometry_msgs::msg::PoseArray::SharedPtr frontiers;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      frontiers = latest_frontiers_;
    }

    if (frontiers && !frontiers->poses.empty()) {
      std::optional<size_t> best_opt = selectBestFrontier(*frontiers, rx, ry, frame_id);
      if (best_opt) {
        size_t best = *best_opt;
        double gx = frontiers->poses[best].position.x;
        double gy = frontiers->poses[best].position.y;
        double dist = std::hypot(gx - rx, gy - ry);

        RCLCPP_INFO(get_logger(), "[goal] Selected [%zu] (%.2f, %.2f) dist=%.2f m",
          best, gx, gy, dist);

        // Remove the chosen centroid from the cached array so a stale
        // republish does not immediately re-select the same point.
        {
          std::lock_guard<std::mutex> lock(frontiers_mutex_);
          if (latest_frontiers_ && best < latest_frontiers_->poses.size()) {
            latest_frontiers_->poses.erase(latest_frontiers_->poses.begin() + best);
          }
        }

        dispatchNavGoal(gx, gy, frame_id, "frontier");
        return;
      }
      RCLCPP_DEBUG(get_logger(),
        "[select] No valid frontier after filters (min_dist=%.2f, avoid_radius=%.2f)",
        get_parameter("min_frontier_distance").as_double(),
        get_parameter("failed_goal_avoidance_radius").as_double());
    }

    // ── Path 2: bootstrap fallback — drive toward a low-cost direction ──
    // Replaces the legacy in-place sweep. The point is for the robot to
    // actually translate so SLAM grows the map, after which Path 1 will
    // naturally take over again.
    if (get_parameter("enable_fallback_drive").as_bool()) {
      const double cooldown = get_parameter("fallback_cooldown_sec").as_double();
      const bool cooldown_ok = !last_fallback_send_time_ ||
        (now() - *last_fallback_send_time_).seconds() >= cooldown;
      if (cooldown_ok) {
        if (auto fb = selectFallbackGoal(rx, ry); fb) {
          last_fallback_send_time_ = now();
          dispatchNavGoal(fb->x, fb->y, frame_id, "fallback");
          return;
        }
      }
    }

    // ── Path 3: detect "exploration complete" ────────────────────────────
    // We reach here only when BOTH the frontier and the fallback paths
    // failed to produce a goal this tick. Two complementary triggers:
    //
    //   (a) `empty_frontier_streak_` ≥ threshold:
    //       /frontiers has been an empty PoseArray for K consecutive
    //       publishes — frontier_detection has nothing to offer.
    //
    //   (b) `idle_tick_streak_` × period ≥ stalemate_idle_sec:
    //       /frontiers is still publishing centroids but selectBestFrontier
    //       rejects all of them (anti-loop, failed_goal_avoidance,
    //       min_frontier_distance) AND fallback also returns nullopt.
    //       This is the "bot hesitates next to a corner person" failure
    //       mode the operator hit. Without (b) the node logs "No valid
    //       frontiers" forever instead of declaring complete.
    //
    // `++idle_tick_streak_` lives at the very end of timerCallback so it
    // ticks exactly once per missed-dispatch tick. dispatchNavGoal()
    // resets it to 0 on any successful dispatch.
    // Minimum-uptime guard, replacing the old `!visit_history_.empty()` gate.
    // The old gate required a prior SUCCESS, so it was shut in exactly the
    // deadlock it was supposed to break. Time since exploration was enabled is
    // the property we actually want: "has this node had a fair chance yet".
    const double min_expl = get_parameter("min_exploration_time_sec").as_double();
    const bool had_fair_chance = exploration_start_time_ &&
      (now() - *exploration_start_time_).seconds() >= min_expl;
    if (get_parameter("enable_exploration_complete_detection").as_bool() &&
        had_fair_chance)
    {
      const int empty_threshold = static_cast<int>(
        get_parameter("empty_frontier_streak_threshold").as_int());
      const double stalemate_sec =
        get_parameter("stalemate_idle_sec").as_double();
      const double rate_hz = get_parameter("rate").as_double();
      const int stalemate_threshold_ticks = (rate_hz > 0.0)
        ? static_cast<int>(stalemate_sec * rate_hz)
        : std::numeric_limits<int>::max();

      const bool empty_done = empty_frontier_streak_ >= empty_threshold;
      const bool stalemate_done = idle_tick_streak_ >= stalemate_threshold_ticks;

      if (empty_done || stalemate_done) {
        const char * reason = empty_done
          ? "empty /frontiers streak"
          : "stalemate (no goal dispatched for ≥ stalemate_idle_sec)";
        RCLCPP_INFO(get_logger(),
          "[complete-trigger] %s — empty_streak=%d, idle_ticks=%d",
          reason, empty_frontier_streak_, idle_tick_streak_);
        declareExplorationComplete(frame_id);
        return;
      }
    }

    // No goal was dispatched this tick. Increment the stalemate counter
    // so trigger (b) above eventually fires.
    ++idle_tick_streak_;

    logExplorationCompleteIfDue();
  }

  /**
   * @brief One-shot transition into "exploration complete" mode.
   *
   * Sets `exploration_complete_` so the timerCallback short-circuit fires
   * on subsequent ticks, and executes the policy from
   * `on_exploration_complete`:
   *  - "stop"        → just log a clear banner; the robot stays idle.
   *  - "return_home" → dispatch one Nav2 goal back to the recorded start
   *                    pose. After it finishes (success or fail) the robot
   *                    stays idle until new frontiers appear.
   *
   * Idempotent: subsequent calls do nothing while `exploration_complete_`
   * is already true.
   */
  void declareExplorationComplete(const std::string & frame_id)
  {
    if (exploration_complete_) return;
    exploration_complete_ = true;

    const std::string policy =
      get_parameter("on_exploration_complete").as_string();

    RCLCPP_INFO(get_logger(),
      "★ EXPLORATION COMPLETE — %d consecutive empty /frontiers messages, "
      "%zu goals reached. Policy=%s.",
      empty_frontier_streak_, visit_history_.size(), policy.c_str());

    if (policy == "return_home") {
      if (start_pose_) {
        RCLCPP_INFO(get_logger(),
          "[home] Returning to start pose (%.2f, %.2f)",
          start_pose_->x, start_pose_->y);
        dispatchNavGoal(start_pose_->x, start_pose_->y, frame_id, "home");
      } else {
        RCLCPP_WARN(get_logger(),
          "[home] return_home requested but no start pose was ever recorded "
          "(TF lookup never succeeded?) — staying idle here.");
      }
    } else {
      RCLCPP_INFO(get_logger(),
        "[stop] Robot will stay idle. To return to start automatically, set "
        "on_exploration_complete: \"return_home\" in params.yaml. To force "
        "re-exploration publish Bool true on /exploration_enabled when "
        "frontiers are available again.");
    }
  }

  /**
   * @brief Build a NavigateToPose goal at (gx, gy) with identity orientation
   *        and dispatch it asynchronously, wiring response/result callbacks.
   *
   * Used by both the frontier path and the fallback-drive path so the FSM
   * transitions, timeout bookkeeping, and failure recording stay identical
   * regardless of where the goal came from. `source` is a short tag
   * ("frontier" / "fallback") for log readability.
   */
  void dispatchNavGoal(double gx, double gy, const std::string & frame_id, const char * source)
  {
    // Reset stalemate counter on any successful dispatch (regardless of
    // path). The counter only ticks up when timerCallback reaches its
    // tail without dispatching, so this is the single canonical place
    // to clear it.
    idle_tick_streak_ = 0;

    NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = frame_id;
    goal.pose.pose.position.x = gx;
    goal.pose.pose.position.y = gy;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;

    current_goal_x_ = gx;
    current_goal_y_ = gy;

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    const std::string source_tag(source);
    send_opts.goal_response_callback = [this, source_tag](typename GoalHandle::SharedPtr gh) {
      if (gh) {
        RCLCPP_INFO(get_logger(), "[action] Goal accepted (%s)", source_tag.c_str());
        current_goal_handle_ = gh;
        goal_sent_time_ = now();
        state_ = State::NAVIGATING;
      } else {
        RCLCPP_WARN(get_logger(), "[action] Goal rejected (%s)", source_tag.c_str());
        recordFailedGoal(current_goal_x_, current_goal_y_);
        state_ = State::IDLE;
      }
    };
    send_opts.result_callback = [this, source_tag](const GoalHandle::WrappedResult & result) {
      current_goal_handle_.reset();
      goal_sent_time_.reset();
      state_ = State::IDLE;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "[result] SUCCEEDED (%s)", source_tag.c_str());
          // Deliberately NOT clearing failed_goals_ here -- see recordFailedGoal.
          recordSuccessfulVisit(current_goal_x_, current_goal_y_);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(get_logger(), "[result] ABORTED (%s)", source_tag.c_str());
          recordFailedGoal(current_goal_x_, current_goal_y_);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "[result] CANCELED (%s)", source_tag.c_str());
          recordFailedGoal(current_goal_x_, current_goal_y_);
          break;
        default:
          RCLCPP_WARN(get_logger(), "[result] Unknown code (%s)", source_tag.c_str());
          recordFailedGoal(current_goal_x_, current_goal_y_);
      }
    };

    state_ = State::NAVIGATING;
    nav_client_->async_send_goal(goal, send_opts);
  }

  /**
   * @brief Cache the latest global costmap snapshot for fallback sampling.
   *
   * Subscribes with TRANSIENT_LOCAL durability (matching Nav2's default) so
   * the very first frame received already reflects the current obstacle
   * picture. Drops zero-sized grids defensively.
   */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) return;
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    current_costmap_ = msg;
  }

  /**
   * @brief Look up the OccupancyGrid cost (0–100) at a world-frame point.
   *
   * @return -1 if costmap is missing or the point falls outside the grid;
   *         otherwise the 8-bit signed cell value (-1 unknown, 0 free, 100 lethal).
   *
   * Used by `selectFallbackGoal` to pick the lowest-cost direction without
   * having to subscribe to the local planner's internal data.
   */
  int8_t getCostAtWorld(double wx, double wy)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!current_costmap_) return -1;
    const auto & cm = *current_costmap_;
    const double ox = cm.info.origin.position.x;
    const double oy = cm.info.origin.position.y;
    const double res = cm.info.resolution;
    if (res <= 0.0) return -1;
    const int mx = static_cast<int>((wx - ox) / res);
    const int my = static_cast<int>((wy - oy) / res);
    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(cm.info.width) ||
        my >= static_cast<int>(cm.info.height))
    {
      return -1;
    }
    return cm.data[static_cast<size_t>(my) * cm.info.width + mx];
  }

  /**
   * @brief Pick a low-cost point on a circle of radius R around the robot.
   *
   * @return The chosen point in map frame, or `nullopt` if no sample qualifies.
   *
   * Strategy:
   *  1. Sample N evenly spaced angles (parameter `fallback_num_samples`).
   *  2. For each, compute the world-frame point at `(rx + R cos θ, ry + R sin θ)`
   *     (`R = fallback_radius_m`).
   *  3. Read its cost from the cached global costmap.
   *  4. Skip cells outside the costmap (cost == -1) and unknown / lethal cells
   *     (cost > `fallback_max_cost`).
   *  5. Honor the same anti-loop and last-failed-goal filters used for
   *     frontier centroids so the bot does not bounce back to a known-bad
   *     spot.
   *  6. Return the candidate with the smallest cost; ties are broken by
   *     iteration order, which is fine because the next tick will resample.
   *
   * If the global costmap has not arrived yet (e.g. very early after launch)
   * this function returns `nullopt`, letting the caller log
   * "exploration complete" instead of busy-spinning.
   */
  std::optional<Point> selectFallbackGoal(double rx, double ry)
  {
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!current_costmap_) {
        // No costmap yet — caller will throttle the "no valid frontier" log.
        return std::nullopt;
      }
    }

    const double radius = get_parameter("fallback_radius_m").as_double();
    const int n_samples = static_cast<int>(get_parameter("fallback_num_samples").as_int());
    const int max_cost = static_cast<int>(get_parameter("fallback_max_cost").as_int());
    if (radius <= 0.0 || n_samples <= 0) return std::nullopt;

    const double avoid_r = get_parameter("failed_goal_avoidance_radius").as_double();
    const double avoid_r_sq = avoid_r * avoid_r;
    const bool antiloop_on = get_parameter("enable_repeated_centroid_filter").as_bool();
    const double loop_r = get_parameter("repeated_centroid_reject_radius").as_double();
    const int max_visits = static_cast<int>(get_parameter("repeated_centroid_max_visits").as_int());
    const double loop_r_sq = loop_r * loop_r;

    std::optional<Point> best;
    int best_cost = std::numeric_limits<int>::max();

    for (int i = 0; i < n_samples; ++i) {
      const double theta = (2.0 * M_PI * i) / static_cast<double>(n_samples);
      const double gx = rx + radius * std::cos(theta);
      const double gy = ry + radius * std::sin(theta);

      const int cost = static_cast<int>(getCostAtWorld(gx, gy));
      // -1 = unknown / out-of-bounds → skip.
      if (cost < 0 || cost > max_cost) continue;

      // Avoid the discs around all unexpired failed goals (frontier AND
      // fallback), so a failing fallback point is not re-offered either.
      if (isNearFailedGoal(gx, gy, avoid_r_sq)) continue;
      // Anti-loop: skip points whose neighborhood we have already visited too often.
      if (antiloop_on && loop_r > 0.0 && max_visits > 0 &&
          static_cast<int>(countSuccessfulVisitsNear(gx, gy, loop_r_sq)) >= max_visits)
      {
        continue;
      }

      if (cost < best_cost) {
        best_cost = cost;
        best = Point{gx, gy};
      }
    }

    if (best) {
      RCLCPP_INFO(get_logger(),
        "[fallback] Drive to (%.2f, %.2f)  cost=%d  radius=%.2fm  samples=%d",
        best->x, best->y, best_cost, radius, n_samples);
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[fallback] No low-cost sample found around robot (radius=%.2fm, max_cost=%d)",
        radius, max_cost);
    }
    return best;
  }

  /**
   * @brief Remember a world-frame goal position that failed, timed out, or was rejected so selection can avoid it.
   *
   * @param gx Goal x in map frame (meters), same as sent to Nav2.
   * @param gy Goal y in map frame (meters).
   * @return void; appends to (or refreshes an entry in) `failed_goals_`.
   *
   * Pipeline role:
   * - Provides **recovery**: stops selection from handing back a goal that has
   *   just failed, for `failed_goal_ttl_sec`.
   *
   * Implementation summary:
   * 1. Drop expired entries.
   * 2. If an entry already covers (gx, gy), refresh its timestamp instead of
   *    appending, so one repeatedly-failing spot cannot fill the set.
   * 3. Otherwise append; evict oldest beyond `failed_goal_max_entries`.
   *
   * Notes:
   * - frontier and fallback goals share this set and CANNOT overwrite each
   *   other. That mutual overwrite was the deadlock.
   * - Entries are NOT cleared on SUCCEEDED. Clearing on success would re-arm a
   *   chronically failing frontier as soon as any fallback goal happened to
   *   succeed, which is the same bug through a different door. TTL alone
   *   decides when a failed goal becomes eligible again.
   */
  void recordFailedGoal(double gx, double gy)
  {
    pruneFailedGoals();
    const double radius = get_parameter("failed_goal_avoidance_radius").as_double();
    const double r_sq = radius * radius;
    for (auto & f : failed_goals_) {
      const double dx = gx - f.x, dy = gy - f.y;
      if (dx * dx + dy * dy < r_sq) {
        f.stamp = now();
        RCLCPP_DEBUG(get_logger(),
          "[failed] Refreshed (%.2f, %.2f); set=%zu", f.x, f.y, failed_goals_.size());
        return;
      }
    }
    failed_goals_.push_back({gx, gy, now()});
    const size_t cap = static_cast<size_t>(
      std::max<int64_t>(1, get_parameter("failed_goal_max_entries").as_int()));
    while (failed_goals_.size() > cap) {
      failed_goals_.erase(failed_goals_.begin());
    }
    RCLCPP_DEBUG(get_logger(), "[failed] Recorded (%.2f, %.2f) for avoidance; set=%zu",
      gx, gy, failed_goals_.size());
  }

  /** Drop failure records older than `failed_goal_ttl_sec`. */
  void pruneFailedGoals()
  {
    const double ttl = get_parameter("failed_goal_ttl_sec").as_double();
    if (ttl <= 0.0) { failed_goals_.clear(); return; }
    const auto t = now();
    failed_goals_.erase(
      std::remove_if(failed_goals_.begin(), failed_goals_.end(),
        [&](const FailedGoal & f) {
          // Guard against a clock jump backwards (sim time reset): treat a
          // negative age as fresh rather than expiring the whole set.
          const double age = (t - f.stamp).seconds();
          return age >= 0.0 && age > ttl;
        }),
      failed_goals_.end());
  }

  /** True when (gx, gy) lies within `radius_sq` of any unexpired failure. */
  bool isNearFailedGoal(double gx, double gy, double radius_sq)
  {
    pruneFailedGoals();
    for (const auto & f : failed_goals_) {
      const double dx = gx - f.x, dy = gy - f.y;
      if (dx * dx + dy * dy < radius_sq) return true;
    }
    return false;
  }

  /** Count how many successful frontier goals landed within radius (map plane) of (mx, my). */
  size_t countSuccessfulVisitsNear(double mx, double my, double radius_sq) const
  {
    size_t n = 0;
    for (const auto & p : visit_history_) {
      const double dx = mx - p.x;
      const double dy = my - p.y;
      if (dx * dx + dy * dy <= radius_sq) {
        ++n;
      }
    }
    return n;
  }

  void recordSuccessfulVisit(double gx, double gy)
  {
    const size_t k_max_history = 256;
    visit_history_.push_back({gx, gy});
    while (visit_history_.size() > k_max_history) {
      visit_history_.erase(visit_history_.begin());
    }
    RCLCPP_DEBUG(get_logger(), "[antiloop] Recorded reached centroid (%.2f, %.2f), history=%zu",
      gx, gy, visit_history_.size());
  }

  /**
   * @brief Log a throttled INFO message when no valid frontiers are available (exploration appears complete).
   *
   * @param none Uses `exploration_complete_log_interval` parameter and `last_exploration_complete_log_time_`.
   * @return void.
   *
   * Pipeline role:
   * - Gives operators a clear **status** when the robot is idle due to empty or filtered frontier lists, without
   *   flooding logs on every timer tick.
   *
   * Implementation summary:
   * 1. Read `exploration_complete_log_interval` (seconds).
   * 2. If no prior log time, or `now` minus last log ≥ interval, emit INFO and store `now` as last log time.
   *
   * Notes:
   * - Exploration may **resume** automatically when frontier_detection publishes new centroids; the message states this.
   */
  void logExplorationCompleteIfDue()
  {
    double interval = get_parameter("exploration_complete_log_interval").as_double();
    auto now_t = now();
    if (!last_exploration_complete_log_time_ ||
        (now_t - *last_exploration_complete_log_time_).seconds() >= interval) {
      RCLCPP_INFO(get_logger(), "[exploration] No valid frontiers — exploration complete (will retry when new frontiers appear)");
      last_exploration_complete_log_time_ = now_t;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr warmup_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_enabled_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  bool exploration_enabled_{true};
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseArray::SharedPtr latest_frontiers_;
  std::mutex frontiers_mutex_;

  // Cached global costmap snapshot used by the fallback drive sampler.
  // Updated in costmapCallback; read in getCostAtWorld / selectFallbackGoal.
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
  std::mutex costmap_mutex_;

  State state_;
  typename GoalHandle::SharedPtr current_goal_handle_;
  std::optional<rclcpp::Time> goal_sent_time_;
  double current_goal_x_{0.0};
  double current_goal_y_{0.0};

  // Point is forward-declared at the top of the private section.
  // Time-limited set of goals that failed, timed out, or were rejected.
  // Entries expire after `failed_goal_ttl_sec`, which is what lets a goal
  // that failed only because the map was still unknown become selectable
  // again once SLAM has grown.
  struct FailedGoal { double x, y; rclcpp::Time stamp; };
  std::vector<FailedGoal> failed_goals_;
  // Set when exploration_enabled_ first turns true; the min-uptime guard for
  // exploration-complete detection is measured from here.
  std::optional<rclcpp::Time> exploration_start_time_;
  std::optional<rclcpp::Time> last_exploration_complete_log_time_;
  // Throttle for fallback dispatches; prevents re-sending the same point
  // every timer tick while Nav2 is rejecting/aborting it.
  std::optional<rclcpp::Time> last_fallback_send_time_;

  // First successful map-frame robot pose; used as the "home" target by
  // the return_home policy on exploration complete.
  std::optional<Point> start_pose_;
  // Counter of consecutive `/frontiers` messages with zero poses; updated
  // in frontiersCallback. When this crosses
  // `empty_frontier_streak_threshold` AND the robot has already reached at
  // least one goal, timerCallback declares exploration complete.
  int empty_frontier_streak_{0};
  // Counter of consecutive timerCallback ticks where neither the frontier
  // path nor the fallback path managed to dispatch a goal. Cleared by
  // dispatchNavGoal() on any successful dispatch. Used to break the
  // "frontiers exist but every candidate is filtered" stalemate where
  // empty_frontier_streak_ would never grow.
  int idle_tick_streak_{0};
  // Set true by declareExplorationComplete; cleared by frontiersCallback
  // when a non-empty PoseArray arrives. Gates the timer short-circuit.
  bool exploration_complete_{false};

  std::vector<Point> visit_history_;

  bool require_startup_warmup_{true};
  bool startup_warmup_done_{false};
  rclcpp::Time warmup_watch_start_;
  bool warmup_timeout_announced_{false};
  std::optional<rclcpp::Time> last_warmup_waiting_log_;
};

}  // namespace tb3_frontier_exploration

/**
 * @brief Program entry: initialize ROS 2 and spin one `GoalAssignmentNode` until shutdown.
 *
 * @param argc Standard C argument count.
 * @param argv Standard C argument vector (ROS 2 remapping may apply).
 * @return 0 after `rclcpp::shutdown`.
 *
 * Pipeline role:
 * - Starts the goal-assignment executable so the exploration stack can receive Nav2 action results and frontier updates.
 *
 * Implementation summary:
 * 1. `rclcpp::init(argc, argv)`.
 * 2. `rclcpp::spin` on a shared `GoalAssignmentNode`.
 * 3. `rclcpp::shutdown()`; return 0.
 *
 * Notes:
 * - Single-threaded executor: timer, subscriptions, and action callbacks are serialized unless a multi-threaded spinner is used elsewhere.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb3_frontier_exploration::GoalAssignmentNode>());
  rclcpp::shutdown();
  return 0;
}
