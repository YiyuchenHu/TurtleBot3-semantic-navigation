# goal_assignment_node — Testing Notes

## Dependencies

- `/frontiers` (geometry_msgs/PoseArray, map frame) published by `frontier_detection_node`
- `/odometry/filtered` (nav_msgs/Odometry) provided by robot_localization or the simulation
- TF: `map` → `base_link` (provided by SLAM + odom → base_link)
- Nav2's `navigate_to_pose` action server is running

## Build

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb3_frontier_exploration
source install/setup.bash
```

## Launch order (consistent with TB3_EXPLORATION_RUN_ORDER)

1. Start the simulation or the real robot (TurtleBot3)
2. Start slam_toolbox (publishes `/map` and `map` → `odom`)
3. Start Nav2 (provides `navigate_to_pose`)
4. Start frontier_detection_node (publishes `/frontiers`)
5. Start goal_assignment_node (with the parameter file)

```bash
ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file src/tb3_frontier_exploration/config/params.yaml
```

Alternatively, load `params.yaml` via a launch file and only run the node.

## Log checks

- **Received frontiers: N** — received N frontier centroids
- **Selected goal [i]: (x, y) dist=d m** — the currently selected nearest frontier and its distance
- **Action goal accepted** / **Action goal rejected** — the goal was accepted or rejected by Nav2
- **Goal finished: SUCCEEDED** / **ABORTED** / **CANCELED** — navigation result

## Quick verification

```bash
# Check whether /frontiers has data
ros2 topic echo /frontiers --once

# Confirm navigate_to_pose exists
ros2 action list | grep navigate_to_pose

# Watch the goal_assignment logs
ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file src/tb3_frontier_exploration/config/params.yaml
```

On success: the robot moves toward the nearest frontier one goal at a time, and after arriving it automatically selects and sends the next goal.
