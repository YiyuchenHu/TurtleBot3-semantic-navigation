# tb3_frontier_exploration

TurtleBot3 frontier-based autonomous exploration package, plus the Gazebo
worlds and simulation launch files used by the rest of the stack.

## Nodes

- **frontier_detection_node** (C++) — Detects frontier cells in `/map`, clusters them, filters the cluster centroids against `/global_costmap/costmap`, and publishes the safe centroids on `/frontiers` (plus RViz markers on `/frontiers_markers`).
- **goal_assignment_node** (C++) — Selects the next frontier and sends goals via the Nav2 `navigate_to_pose` action. With `start_enabled: false` (the default in `config/params.yaml`) it stays idle until `true` arrives on `/exploration_enabled`, which the coordinator publishes when a search starts.
- **startup_map_warmup_node.py** — Once exploration is enabled, turns the robot about 45° left and right in place to seed the SLAM map, then publishes `exploration_warmup_complete`; `goal_assignment_node` waits for that signal (up to `startup_warmup_timeout_sec`) when `require_startup_warmup` is true.

## Launch

- `frontier_detection.launch.py` — Run frontier detection only.
- `goal_assignment.launch.py` — Run goal assignment only.
- `exploration.launch.py` — Run the warm-up node, frontier detection and goal assignment. Pass `start_enabled:=true` to explore without a coordinator; the warm-up node still waits for `/exploration_enabled`, so goal assignment then starts after the warm-up timeout (90 s by default).

## Dependencies

ROS 2 Humble, Nav2, slam_toolbox (or another map source). See `package.xml` for message/action dependencies.

## Config

`config/params.yaml` — Parameters for both C++ nodes (topics, frame_id, costmap filter, timeouts, fallback drive, exploration-complete detection).

## Docs

- [`docs/TEST_FRONTIER_DETECTION.md`](docs/TEST_FRONTIER_DETECTION.md) — clustering, parameters, build/run and RViz checks.
- [`docs/COSTMAP_FILTER_AND_DEBUG.md`](docs/COSTMAP_FILTER_AND_DEBUG.md) — costmap filtering and grid/world conversion.
- [`docs/FRONTIER_DETECTION_DEBUG_LOGS.md`](docs/FRONTIER_DETECTION_DEBUG_LOGS.md) — reading the step-by-step debug logs.
