# tb3_bringup/launch

Every launch file in the course lives here. Nothing here implements a node —
these files only start nodes that live in the `tb3_*` packages and point them
at the right config, world and model files.

## The six-terminal flow

| File | Terminal | Starts | Common edits live in |
|---|---|---|---|
| `sim.launch.py` | T1 | Gazebo (gzserver + gzclient), robot_state_publisher, TurtleBot3 spawn | `world:=` arg + `WORLD_PRESETS` in this file; worlds in `tb3_bringup/worlds/`, the models they reference in `tb3_bringup/models/` |
| `nav.launch.py` | T2 | `nav2_bringup` with `slam:=True` (starts slam_toolbox), RViz | upstream `nav2_params.yaml`; RViz layout in `tb3_bringup/rviz/semantic_nav.rviz` |
| `backend.launch.py` | T3 | semantic memory, semantic map memory, query, nav adapter, coordinator, warm-up + frontier exploration (8 nodes) | each node's own package: `tb3_memory/config/`, `tb3_coordinator/config/`, `tb3_query/config/`, `tb3_nav_adapter/config/`, `tb3_frontier_exploration/config/`. The semantic target registry is the exception — it is an asset, so it lives in `tb3_bringup/config/semantic_targets.yaml` and this file passes its path to `semantic_query_node` |
| `localizer.launch.py` | T4 | `localizer_node` | `tb3_localizer/config/localizer.yaml` |
| `detector.launch.py` | T5 | `detector_node` | `tb3_detector/config/detector.yaml`; weights in `tb3_detector/models/` |

T6 is a plain shell — no launch file — used to publish `/user_command`.

## Not part of the six-terminal flow

| File | Purpose |
|---|---|
| `full_stack.launch.py` | All five of the above in one process with fixed start-up delays. For demos and smoke tests; wrong for development, because one log stream and no per-subsystem restart. |
| `detector_test.launch.py` | Static self-test world (`detector_test.world`) for the Stage-1 YOLO check in INSTRUCTIONS Step 4. Gazebo only — no SLAM, Nav2 or course nodes. |

## Which file do I edit?

- **Change the world / spawn pose** → `sim.launch.py` (`WORLD_PRESETS`)
- **Turn RViz off** → `nav.launch.py use_rviz:=false`
- **Tune the detector** → `tb3_detector/config/detector.yaml`, not a launch file
- **Tune exploration** → `tb3_frontier_exploration/config/params.yaml`
- **Add or rename a semantic target** → `tb3_bringup/config/semantic_targets.yaml`
- **Turn on the runtime debug overlay** → `backend.launch.py use_runtime_debug:=true`
