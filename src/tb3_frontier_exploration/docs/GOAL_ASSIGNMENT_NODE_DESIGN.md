# goal_assignment_node — Design

Practical ROS2 C++ design for selecting frontier goals and sending them to Nav2.

---

## 1. Node responsibilities

- **Subscribe** to `/frontiers` (geometry_msgs/PoseArray) and `/odometry/filtered` (nav_msgs/Odometry).
- **Store** latest frontiers and robot pose (in map frame); optionally transform odom to map if needed.
- **Select** one goal from the frontier list using a configurable strategy (nearest, largest, information-gain proxy).
- **Send** the selected pose to Nav2 via the **NavigateToPose** action client.
- **Handle** action result: on success → pick next goal and send again; on failure/cancel → pick another frontier (or retry) and re-send; avoid re-sending the same goal until it is done or replaced.
- **Throttle** so we do not send a new goal while one is already in progress (unless we explicitly cancel).

---

## 2. Subscriptions and action client structure

| Type | Name | Topic / action | Message / type |
|------|------|----------------|----------------|
| Subscription | `frontiers_sub_` | `frontiers_topic` (e.g. `/frontiers`) | geometry_msgs/msg/PoseArray |
| Subscription | `odom_sub_` | `odom_topic` (e.g. `/odometry/filtered`) | nav_msgs/msg/Odometry |
| Action client | `nav_client_` | `navigate_to_pose_action` (e.g. `navigate_to_pose`) | nav2_msgs/action/NavigateToPose |

**Data flow:**

- **frontiers callback:** Store latest PoseArray (and header) in a member; do **not** send a goal from here (to avoid sending on every frontier update). Optionally trigger a one-shot or timer if you want to re-evaluate when frontiers change.
- **odom callback:** Store robot pose. If odom is in `odom` frame, you need TF (map → odom) to get robot pose in `map` frame for distance/selection. Alternatively use `/odometry/filtered` if it is already in map frame, or use TF in a timer to resolve robot pose in map.
- **Timer or state machine:** At a fixed rate (e.g. 1 Hz) or when the previous goal finishes: if no goal in progress, run **selectGoal()** → **sendGoal(pose)**. So the “driver” is either a timer (“try to send next goal”) or the action result callback (“goal finished, send next”).

**Recommended:** Keep a **state**: `IDLE` (no goal in progress) vs `NAVIGATING` (goal sent, waiting for result). When in IDLE, a timer (or the result callback) calls `selectAndSendNextGoal()`. When sending a goal, set state to NAVIGATING. On result (success/failure/cancel), set state to IDLE and then call `selectAndSendNextGoal()` again (or let the timer do it).

---

## 3. Goal selection strategy candidates

Assume robot pose in map frame is `(rx, ry)` and frontiers are `PoseArray.poses[i].position.x/y`.

**A. Nearest frontier**

- For each pose in the PoseArray, compute distance `d = sqrt((x-rx)^2 + (y-ry)^2)`.
- Choose the index with smallest `d`.
- **Pros:** Simple, fast, tends to reduce travel time. **Cons:** May ignore large unexplored regions.

**B. Largest cluster**

- PoseArray does not carry cluster size. To use “largest,” you would need a custom message (e.g. array of poses + array of sizes) or a separate topic with cluster sizes. If you only have PoseArray, you can **approximate** by preferring frontiers that are far from other frontiers (proxy for “big” frontier region): e.g. choose the pose whose minimum distance to other poses is largest.
- **Alternative (with PoseArray only):** “Farthest frontier” — choose the pose with **largest** distance to robot. Encourages exploration toward distant frontiers.

**C. Information gain proxy**

- Approximate “information gain” by distance and local density: e.g. `score = 1/d + lambda * (min distance to other frontiers)`. Tune `lambda` so that closer frontiers are preferred but isolated frontiers (high min distance to others) get a boost.
- Or: `score = cluster_size / distance` if you have cluster size; with PoseArray only, use a proxy (e.g. number of other frontiers within radius R) as a stand-in for “importance.”

**Recommended for first implementation:** **Nearest frontier** (strategy A). Add a parameter `strategy: "nearest" | "farthest" | "largest_proxy"` and implement nearest first; extend later.

---

## 4. State variables to store

| Variable | Type | Purpose |
|----------|------|--------|
| `latest_frontiers_` | geometry_msgs::msg::PoseArray::SharedPtr (or copy) | Latest frontier poses; header gives frame_id and stamp. |
| `frontiers_mutex_` | std::mutex | Protect `latest_frontiers_` from concurrent callback updates. |
| `robot_pose_x_`, `robot_pose_y_` | double | Robot position in map frame (for selection). |
| `robot_pose_valid_` | bool | True once we have received at least one odom (or TF). |
| `goal_in_progress_` | bool | True after send_goal_async, false after result (success/failure/cancel). Prevents sending a second goal while Nav2 is still working. |
| `current_goal_index_` | size_t (optional) | Index of the frontier we last sent; used to avoid immediately re-sending the same goal if the list hasn’t changed (see section 5). |
| `action_server_ready_` | bool | True after nav_client_->wait_for_action_server(). Used to avoid sending before Nav2 is up. |

Optional for “largest” / “information gain”:

- If you add a custom message with cluster sizes: store and use them in the scoring function.

---

## 5. Avoiding repeatedly sending duplicate goals

- **One goal at a time:** Only send a new goal when `!goal_in_progress_`. Set `goal_in_progress_ = true` when you call `async_send_goal`; set it to `false` in the result callback (success, failure, canceled).
- **Don’t re-send the same pose:** When you select a frontier index `i`, record it (e.g. `current_goal_index_ = i`). After the goal completes (success or failure), you can either:
  - **Option A:** Remove or “invalidate” that index from consideration for the next N seconds (e.g. a “recently sent” set with timestamps), or
  - **Option B:** Simply select again from the full list; the next selection might be the same if it’s still nearest. To avoid that, **exclude the last sent index** for the next selection (e.g. “nearest excluding index `current_goal_index_`”), or exclude poses within a small distance of the last sent pose.
- **Distance threshold:** Before sending a goal, check that the selected pose is not within `min_goal_dist` of the last sent pose (store `last_sent_x_, last_sent_y_`). If the list is unchanged and the robot hasn’t moved much, the same frontier might still be nearest; a small distance threshold (e.g. 0.5 m) avoids sending the same goal again and again.
- **Summary:** Use `goal_in_progress_` to enforce one goal at a time; use “exclude last sent” or “min distance from last sent” to avoid duplicate goals in quick succession.

---

## 6. Handling success, failure, and canceled goals

**NavigateToPose** result is received in the goal result callback (e.g. `handle_result`).

- **Success (SUCCEEDED):**
  - Set `goal_in_progress_ = false`.
  - Optionally log “Goal reached.”
  - Call `selectAndSendNextGoal()` (or set a flag so the timer does it) to send the next frontier. Do not re-send the same goal (use exclusion or min distance as above).

- **Failure (ABORTED / FAILED):**
  - Set `goal_in_progress_ = false`.
  - Log warning (and optionally the result code).
  - Call `selectAndSendNextGoal()` to pick another frontier (e.g. next nearest, or exclude the failed one for a short time). No need to retry the same goal immediately; the frontier list will change as the map updates.

- **Canceled (CANCELED):**
  - Set `goal_in_progress_ = false`.
  - Either call `selectAndSendNextGoal()` to send a new goal, or do nothing and let the user re-enable exploration. Typically treat like failure: pick next goal.

**Implementation detail:** Use `rclcpp_action::Client<NavigateToPose>::SendGoalOptions`. Set `result_callback` to the handler above; optionally `goal_response_callback` to check if the goal was accepted. In the result callback, switch on `result.code` (SUCCEEDED, ABORTED, CANCELED) and update state + trigger next goal as above.

---

## 7. Suggested class skeleton (C++)

```cpp
// State
enum class State { IDLE, NAVIGATING };

geometry_msgs::msg::PoseArray::SharedPtr latest_frontiers_;
std::mutex frontiers_mutex_;
double robot_pose_x_{0.0}, robot_pose_y_{0.0};
bool robot_pose_valid_{false};
std::atomic<State> state_{State::IDLE};
std::string frame_id_;
double last_sent_x_{0.0}, last_sent_y_{0.0};
bool last_sent_valid_{false};
double min_goal_separation_;  // param: min distance from last sent to avoid duplicates

// In frontiers callback: store latest_frontiers_ (under mutex).
// In odom callback: update robot pose (map frame via TF or from msg if already map).
// Timer or result callback: if state_ == IDLE, selectAndSendNextGoal().
// selectAndSendNextGoal(): lock frontiers, select index (nearest/farthest), 
//   check min_goal_separation from (last_sent_x_, last_sent_y_), build goal pose,
//   send via nav_client_->async_send_goal(..., result_callback), set state_ = NAVIGATING.
// In result_callback: set state_ = IDLE; update last_sent_* to the goal we sent; call selectAndSendNextGoal().
```

---

## 8. Parameters (suggested)

| Parameter | Type | Default | Description |
|-----------|------|--------|-------------|
| `frontiers_topic` | string | `/frontiers` | PoseArray topic. |
| `odom_topic` | string | `/odometry/filtered` | Odometry topic. |
| `navigate_to_pose_action` | string | `navigate_to_pose` | Action name. |
| `frame_id` | string | `map` | Frame for goal pose and robot. |
| `strategy` | string | `nearest` | `nearest` \| `farthest` \| (future: `largest_proxy`). |
| `min_goal_separation` | double | 0.5 | Min distance (m) from last sent goal to avoid duplicates. |
| `goal_timeout` | double | 60.0 | Action goal timeout (s). |
| `rate` | double | 1.0 | Hz for timer that tries to send next goal when IDLE. |

This design is ready to implement in ROS2 Humble C++ with rclcpp and rclcpp_action.
