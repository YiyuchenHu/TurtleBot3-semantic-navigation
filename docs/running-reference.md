# Reference: build details, worlds, and where to change things

The detail behind [README -> Setup by platform](../README.md#setup-by-platform)
and [README -> Running](../README.md#running). Read those first; this page is
what you reach for once the basics work.

## Build details

**What the apt list already covers.** `ros-humble-desktop` already pulls
`tf2-geometry-msgs`, `rviz2` and `xacro`, and the Gazebo 11 runtime comes with
`gazebo-ros-pkgs`; the explicit entries in the README's list just make the
`package.xml` dependencies visible.

**Exact validated versions.** The course is validated on Ubuntu 22.04 /
Python 3.10.12 with `torch 2.5.1+cpu`, `torchvision 0.20.1+cpu`,
`ultralytics 8.4.31`, `numpy 1.26.4` and `cv2 4.9.0`; `opencv-python` is pinned
to the version this repository was validated with, and `python3-yaml` comes
with ROS.

**Optional COCO weights.** The shipped `tb3det_yolo26n.pt` is a fine-tuned
model. The COCO `yolo26n.pt` it replaced is only needed to compare the two
(git-ignored):

```bash
wget -O src/tb3_detector/models/yolo26n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
```

**Rebuilding one package.** After the first full build:

```bash
colcon build --symlink-install --packages-select tb3_detector
```

Keep `--symlink-install` on every build. The first build set it, and dropping
it later copies files into `install/` that the symlinked build expects to be
links, so edits to configs and launch files silently stop taking effect.

**Gazebo models need no network.** The models the course worlds use (person,
trash can, chair, …) are **vendored in this repository**, in
`src/tb3_bringup/models/` next to the worlds that reference them in
`src/tb3_bringup/worlds/` (see [NOTICE](../NOTICE)), and are wired up via
`GAZEBO_MODEL_PATH` inside the launch files — no online model database access
is needed.

**Generated directories.** `build/`, `install/` and `log/` are written by
`colcon build` from `src/`; `install/` holds symlinks back into `src/` (that is
what `--symlink-install` does), so an edit made there is either overwritten on
the next build or silently editing the source through a link. Do not read or
edit them. All three are git-ignored; deleting them and rebuilding is a safe
reset.

## Choosing a world

| `world:=` alias | File | Contents | Auto spawn |
|---|---|---|---|
| `warehouse_models_person` *(default)* | `warehouse_models_person.world` | 6×6 m room, 5 person figures (corners + centre) | `(-1.5, 0.0)` |
| `warehouse_models` | `warehouse_semantic_models.world` | 4×6 m room, 1 person + 1 trash can + 1 chair (all three semantic targets) | `(-1.2, -1.2)` |

The world is picked in T1 (everything else is world-agnostic):

```bash
ros2 launch tb3_bringup sim.launch.py world:=warehouse_models
```

An absolute path to a custom `.world` file is also accepted. The same
`world:=` argument works on the one-command launch below.

## One-command launch

For demos and smoke tests:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_bringup full_stack.launch.py
```

Give it ~30 s to settle. It forwards `world:=`, `use_rviz:=` and
`use_runtime_debug:=`. For day-to-day work use the six-terminal flow above.

For an unattended end-to-end acceptance run (clean restart, the six terminals in
tmux, the navigation commands, and a pass/fail report), run
[`scripts/acceptance_run.sh`](../scripts/acceptance_run.sh). It needs `tmux`, and
defaults to `--world warehouse_models`, the world holding all three targets;
`--dry-run` prints the plan without starting anything.

## Where to change things

Four files cover almost every edit. Everything is under `src/`.

| I want to … | Edit | Notes |
|---|---|---|
| **Use a different world** | `src/tb3_bringup/worlds/*.world`, and `WORLD_PRESETS` in `src/tb3_bringup/launch/sim.launch.py` to give it a `world:=` alias and a spawn pose | Objects the world places must exist in `src/tb3_bringup/models/`; add a new prop there first |
| **Add or rename a semantic target** | `src/tb3_bringup/config/semantic_targets.yaml` | The one registry mapping `semantic_name` ↔ `detector_label` ↔ Gazebo model. Add the detector label to `class_filter` in `detector.yaml` too |
| **Tune detector thresholds** | `src/tb3_detector/config/detector.yaml` | `conf_threshold`, `class_filter`, `device`. `class_filter` takes detector labels (`"trash_can"`, or `"traffic light"` with the COCO weights), never semantic names |
| **Tune exploration / goal assignment** | `src/tb3_frontier_exploration/config/params.yaml` | Frontier size, goal spacing, blacklist TTL. The thresholds interlock, so change one at a time |
| **Tune memory, query, approach pose** | `src/tb3_memory/config/`, `src/tb3_query/config/`, `src/tb3_nav_adapter/config/`, `src/tb3_coordinator/config/` | One YAML per package, named after the node it configures |
| **Change RViz layout** | `src/tb3_bringup/rviz/semantic_nav.rviz` | Opened by `nav.launch.py`; `use_rviz:=false` turns it off |
| **Change which nodes start** | `src/tb3_bringup/launch/` | One launch file per terminal; `src/tb3_bringup/launch/README.md` maps each to its terminal |
| **Change detection code** | `src/tb3_detector/tb3_detector/detector_core.py` | `load()` and `infer()` |

**`build/`, `install/` and `log/` are generated — do not read them and do not
edit them.** `colcon build` writes all three from `src/`; `install/` holds
symlinks back into `src/` (that is what `--symlink-install` does), so an edit
made there is either overwritten on the next build or silently editing the
source through a link. Anything you actually want to keep goes in `src/`.
All three are git-ignored; deleting them and rebuilding is a safe reset.
