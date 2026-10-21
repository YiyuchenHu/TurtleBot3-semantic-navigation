# goal_assignment_node — State Machine and Debugging Notes

## State machine

```
                    +------------------+
                    |       IDLE      |
                    | (no goal active) |
                    +--------+--------+
                             |
        valid frontiers + action server ready
        pick nearest frontier passing filters, send goal
                             v
                    +------------------+
                    |    NAVIGATING    |
                    | (goal sent, wait) |
                    +--------+--------+
                             |
        +---------------------+----------------------+
        |                     |                      |
   SUCCEEDED             ABORTED/CANCELED          timeout
   (clear last_failed)   (record failed point,     (cancel goal,
                          avoid re-selecting)       record failed point)
        |                     |                      |
        v                     v                      v
                    +------------------+
                    |       IDLE       |
                    +------------------+
```

- **IDLE**: no navigation goal is currently being executed; each cycle may try to select and send the next goal.
- **NAVIGATING**: a goal has been sent to Nav2; waiting for the result or a timeout.
  - If `now - goal_sent_time >= goal_timeout`, actively cancel the goal, record it as a failed point, and return to IDLE.
  - After receiving a result (SUCCEEDED / ABORTED / CANCELED), return to IDLE; on failure or cancellation, also record the goal position as the "last failed goal".

## Implementation logic summary

1. **Filter "too close" frontiers**  
   Centroids closer to the robot than `min_frontier_distance` are excluded from the "nearest" selection, avoiding useless goals.

2. **Avoid re-sending failed goals**  
   Record the coordinates of the most recent failed/canceled/timed-out goal; during selection, exclude frontiers within `failed_goal_avoidance_radius` of the "last failed goal".  
   Only clear that failed point after a goal **SUCCEEDED** (optional policy: it could also be cleared after sending a new goal).

3. **Timeout handling**  
   In NAVIGATING, if `goal_timeout` seconds elapse from `goal_sent_time` without a result, call `async_cancel_goal` to cancel the current goal, record it as a failed point, and return to IDLE; the next cycle will re-select a goal.

4. **No valid frontiers**  
   If, after the "too close" and "near failed point" filters, no candidates remain, treat it as having no valid goal;  
   use `exploration_complete_log_interval` to rate-limit the "No valid frontiers — exploration complete" message and avoid log spam.

5. **Only one goal at a time**  
   New goals can only be sent while IDLE; during NAVIGATING, only the timeout check runs while waiting for the result.

## Suggested parameters

| Parameter | Type | Suggested value | Description |
|------|------|--------|------|
| `min_frontier_distance` | double | 0.5 | Ignore frontiers closer to the robot than this distance (meters). |
| `failed_goal_avoidance_radius` | double | 1.0 | Do not select frontiers within this distance of the "last failed goal" (meters). |
| `goal_timeout` | double | 60.0 | Maximum execution time per goal; on timeout, cancel and record as failed (seconds). |
| `exploration_complete_log_interval` | double | 5.0 | Minimum interval between "exploration complete"-type log messages (seconds). |
| `rate` | double | 1.0 | Main loop / timer frequency (Hz). |

## Debug logging recommendations

- **At startup**: `min_frontier_dist`, `failed_avoid_radius`, and `goal_timeout` are already printed, making it easy to confirm the configuration.
- **During normal operation**:  
  - Use **INFO** for: goal accepted/rejected, selected goal index and coordinates, results (SUCCEEDED/ABORTED/CANCELED), timeout cancellations, and the rate-limited "exploration complete" message.  
  - Use **DEBUG** for: the number of frontiers received each time, candidates filtered out as "too close" or "near failed point", TF failures, and recorded failed-point coordinates.  
- **Troubleshooting "no goal selected"**:  
  - Set the log level to DEBUG:  
    `ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file ... --log-level goal_assignment_node:=DEBUG`  
  - Check for frequent "[select] Frontier [i] skipped: too close" or "near last failed goal" messages, as well as "[select] No valid frontier after filters".
- **Troubleshooting timeouts**:  
  - Check whether `goal_timeout` is too small or the environment makes planning/execution slow;  
  - Watch whether "[timeout] Goal stalled for Xs (limit Ys), canceling" appears frequently.

## Log tag meanings

- `[frontiers]` — subscribed frontier data.
- `[TF]` — robot pose lookup.
- `[select]` — candidate filtering and nearest selection.
- `[goal]` — the final selected goal.
- `[action]` — goal accepted/rejected.
- `[result]` — navigation result.
- `[timeout]` — timeout cancellation.
- `[failed]` — recording a failed point (DEBUG).
- `[exploration]` — rate-limited message when no valid frontiers remain.
- `[idle]` — reason while in IDLE (DEBUG, e.g. action server not ready).
