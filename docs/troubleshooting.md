# Troubleshooting

Symptoms and fixes for the simulation itself, on every platform. Setup
problems specific to one host belong on that host's page:
[macOS](setup-macos.md), [Windows](setup-windows.md).

## Clean restart

`Ctrl-C` normally shuts a terminal's launch down cleanly, but a killed
terminal or a crashed Gazebo can leave orphan processes behind. Symptoms
and the matching fix, in order:

1. **T1 won't start / `spawn_entity` hangs on `Waiting for service
   /spawn_entity` / Gazebo complains the master port is in use** — an old
   Gazebo is still running:
   ```bash
   pkill -9 -x gzserver; pkill -9 -x gzclient
   ```
2. **`ros2 node list` shows nodes you did not start (or duplicates), the
   robot chases exploration goals with everything closed** — orphan
   course/Nav2 nodes survive:
   ```bash
   pkill -9 -f 'detector_node|localizer_node|coordinator_node|semantic_memory_node|semantic_map_memory_node|semantic_query_node|nav_goal_adapter_node|frontier_detection_node|goal_assignment_node|startup_map_warmup'
   pkill -9 -f 'slam_toolbox|nav2|lifecycle_manager|bt_navigator|controller_server|planner_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother|map_saver'
   pkill -9 -x rviz2; pkill -9 -x robot_state_publisher
   ```
3. **Ghost topics/nodes still listed after everything is dead** — stale
   discovery cache:
   ```bash
   ros2 daemon stop        # restarts automatically on the next ros2 command
   ```

Then re-open T1–T5 in order as above. Verify the slate is clean with
`ros2 node list` (should be empty or error out).

## Robot spins on the spot and never drives

**Symptom.** The robot rotates in place indefinitely instead of exploring,
and T2 repeats:

```text
[controller_server] [ERROR] Failed to make progress
[controller_server] [WARN]  [follow_path] [ActionServer] Aborting handle.
```

Frontier goals keep being accepted, `/cmd_vel` carries a non-zero
`angular.z` with `linear.x` stuck at 0, and nothing is actually in the way —
`ros2 topic echo /scan` shows metres of clear space all round.

**This is not caused by the detector.** It is an occasional wedge in the Nav2
controller/recovery loop, and it
can occur before the detector has published anything at all. Nothing in
`detector_core.py` can cause it or fix it — the detector is not in the
control loop.

**Fix.** `Ctrl-C` every terminal and run the clean-restart procedure
above, then re-open T1–T5 in order. It clears on restart. The map and any
landmarks built so far are lost, so the robot re-explores from scratch.

Do **not** try to fix this by lowering `conf_threshold` or widening
`class_filter` — they are unrelated; see [NOTES.md](../NOTES.md).

## `go to chair` keeps failing and RViz shows a `trash_can` marker on the chair

**Symptom.** The person and the trash can are found and reached, but the
chair never appears as `chair_0`; instead a second trash-can marker
(`trash_can_1`) sits where the chair is, and `go to chair` ends in
`query failed: no active chair in memory`.

**This is a known detector issue.** From some viewpoints the
shipped detector labels the chair as a trash can with high confidence, and if
that wrong label reaches the promotion threshold first it is what gets
promoted. It happened in 0 of 8 reference runs with the shipped settings, so
it is rare, but it does occur. Details are in [NOTES.md §8.1](../NOTES.md).

**It now corrects itself — wait before restarting.** A landmark keeps a count
per label, so the chair observations that keep arriving are recorded against
the ghost instead of being thrown away, and the landmark is renamed once the
chair count is 1.5x the trash-can count and past chair's own threshold of 12.
Watch T3 for the line:

```text
[relabel] relabeled trash_can_0 -> chair_0 at (-1.34, 2.31): chair 50 vs trash_can 33
```

The RViz marker changes colour and text at the same moment, and `go to chair`
works from then on. The `[gate_summary]` block prints the running counts
(`counts=[chair:45,trash_can:33]`) so you can watch the evidence build.

**If it does not correct.** The correction needs the robot to keep seeing the
chair, so it will not happen while the robot is exploring elsewhere. If the
counts are not moving after the robot has driven past the chair a few times,
run the clean-restart procedure above. Do not raise `candidate_timeout` to
work around this — that makes the ghost more likely, not less.
