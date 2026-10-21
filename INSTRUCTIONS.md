# Assignment: Give the Robot Eyes — Object Detection in ROS 2

## The single file you edit

Implement the two methods `load()` and `infer()` in
`src/tb3_detector/tb3_detector/detector_core.py` — the only file the graded task
requires you to change. Two optional exceptions: tune `conf_threshold` in
`config/detector.yaml` while developing (the acceptance test runs with the
shipped config), and see NOTES.md §9 for bonus work on `tb3_localizer`.
Everything else in the workspace is provided and working.

## Steps

### Step 1 — Install the dependencies

Follow **[README.md](README.md) → Setup by platform**, steps 1–2: apt packages and
`ultralytics` + `torch`. The detector weights (`tb3det_yolo26n.pt`) are
provided with the repository — nothing to download. Rebuild the detector
package once so the weights reach `install/`:

```bash
cd ~/TurtleBot3-semantic-navigation
colcon build --symlink-install --packages-select tb3_detector
```

Keep `--symlink-install` on every build, including single-package ones — the
first full build set it, and dropping it later replaces symlinks in
`install/` with copies, so later config and launch edits stop taking effect.

Set `device: "cuda:0"` in `config/detector.yaml` only if you have a GPU.

### Step 2 — Record the baseline

Run the six-terminal flow (README → Running) with the stub still in place and
note what it does: `detections: []` streaming at camera rate, a box-free debug
image, and `query failed: no active person in memory` on
`/coordinator_node/status`. Full "before" snapshot: NOTES.md §7.1.

### Step 3 — Implement `DetectorCore`

Follow the `TODO(student)` blocks in `detector_core.py`.

- `load()` — import `ultralytics`, load `self.model_path`, move the model to
  `self.device`; raise a clear exception if the package or the weights file is
  missing.
- `infer(bgr_image)` — predict with `self.conf_threshold`, apply
  `self.class_filter`, and return a list of dicts in the exact format given in
  the contract tables below. Return `[]` when nothing passes.
- Do not edit `detector_node.py`. Do not edit `src/tb3_query/msg/`.
- Do not add `"airplane"` to `class_filter`, do not replace the shipped
  whitelist with `[""]`, and never write a bare `[]` for it in YAML.
  See NOTES.md §4.

Development loop — Terminal 5 only (see the README six-terminal table):

1. Edit `detector_core.py`.
2. In T5: `Ctrl-C`, then re-run `ros2 launch tb3_bringup detector.launch.py`.
3. Watch T5's log and the RViz **Detector Debug Image**.

Rebuild only when files are added or removed (e.g. the weights in Step 1).

### Step 4 — Self-test the detector in isolation

```bash
# Terminal 1 — static self-test world
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_bringup detector_test.launch.py

# Terminal 2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_bringup detector.launch.py

# Terminal 3
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /detector_node/detections
ros2 run rqt_image_view rqt_image_view /detector_node/debug_image
```

Expect one box labelled `person` with a sensible confidence. Three props stand
in that world and only `person` is reported — NOTES.md §1.5 says why. If boxes
land in the wrong place, re-check the pixel-coordinate convention below.

### Step 5 — Full-stack run

Bring up the six terminals again and drive the robot to real objects, as in the
acceptance criteria below. T1 picks the world; the others are world-agnostic and
restart unchanged. `scripts/acceptance_run.sh` runs the sequence unattended.

## Interface contract

### Topics (names must not change)

| Direction | Topic | Type | QoS |
|---|---|---|---|
| in | `/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT (Gazebo sensor) |
| in | `/camera/camera_info` | `sensor_msgs/CameraInfo` | BEST_EFFORT |
| out | `/detector_node/detections` | `vision_msgs/Detection2DArray` | RELIABLE |
| out | `/detector_node/debug_image` | `sensor_msgs/Image` | RELIABLE |

Subscribe to the camera with BEST_EFFORT; a RELIABLE subscription receives
nothing and says nothing (NOTES.md §7.6).

### `DetectorCore.infer()` return value — `list[dict]`, one dict per detection

| Key | Type | Value |
|---|---|---|
| `label` | `str` | raw COCO class name, e.g. `"person"` |
| `conf` | `float` | 0..1, e.g. `0.87` |
| `bbox_xyxy` | `list[float]` | `[x1, y1, x2, y2]`, pixels |
| `track_id` | `int` or `None` | `None` unless you enable tracking |

### Pixel-coordinate convention

| Rule | Value |
|---|---|
| Origin `(0, 0)` | top-left of the image |
| Axes | x → right, y → down |
| `(x1, y1)` | top-left corner of the box |
| `(x2, y2)` | bottom-right corner of the box |
| Resolution | pixels of the original `/camera/image_raw` image |
| If you resize before inference | scale boxes back; ultralytics already returns original-resolution boxes |

### Label mapping (`semantic_targets.yaml`)

| Task-level `semantic_name` | `detector_label` your `infer()` must report |
|---|---|
| `person` | `"person"` |
| `trash_can` | `"trash_can"` |
| `chair` | `"chair"` |

Report the label exactly as the network emits it (`result.names[...]`); never
rename it in `infer()`. With the provided fine-tuned weights the two columns
happen to agree; with the optional COCO weights (NOTES.md) they do not — the
trash can comes out as `"traffic light"` — and the mapping in
`semantic_targets.yaml` is what keeps the rest of the stack working either way.
`class_filter` entries are detector labels, not semantic names.

## Acceptance criteria

"Reaches" means the robot ends within **1.2 m of the real object**, measured
against Gazebo ground truth. A `TARGET_REACHED` on its own is not enough.

The 1.2 m is an error budget, not a slack allowance: the nav adapter aims for
a 0.5 m standoff, the landmark itself sits systematically short of the object
centre (the LiDAR returns the near surface, and for the chair that bias
measured 0.15–0.36 m), and Nav2 stops anywhere inside its own xy goal
tolerance. Worst case those stack to roughly 1.1 m with everything working
correctly, which left no headroom under the old 1.0 m bar. The reference run
finished at 0.78 m, 0.62 m and 0.92 m — NOTES.md §6.1 derives the budget.

Your implementation passes when, with `detector_core.py` as the only code change:

1. `colcon build --symlink-install` succeeds and the stack runs without node
   crashes in the six-terminal flow.
2. In `world:=warehouse_models_person`, after exploration has seen the room,
   `go to person` and `go to person N` — for at least two different observed
   `N` — each reach the correct person, and exploration resumes automatically:
   ```bash
   ros2 topic pub --once /user_command std_msgs/String "data: 'go to person'"     # nearest
   ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 1'"   # by ID
   ```
3. In the second world, `go to trash can` and `go to chair` reach their targets
   the same way:
   ```bash
   # Terminal 1 (after a clean restart)
   ros2 launch tb3_bringup sim.launch.py world:=warehouse_models
   ```
   ```bash
   # Terminal 6
   ros2 topic pub --once /user_command std_msgs/String "data: 'go to trash can'"
   ros2 topic pub --once /user_command std_msgs/String "data: 'go to chair'"
   ```
4. RViz shows your detections live: boxes in the **Detector Debug Image** panel
   and sphere+text landmarks (`/semantic_memory_markers`) on the map.
5. No interface drift: topic names, message types and the `infer()` dict format
   are unchanged.

Background and design rationale: see NOTES.md
