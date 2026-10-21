# `tb3_localizer`

Stage 2: 2D detection → position. The bounding-box centre column gives a bearing through the camera's horizontal FOV, the LiDAR scan at that bearing gives a range, and the two combine into an `(x, y)` in `base_link`. The LiDAR ranges the object's near surface rather than its centre, so the result is biased toward the robot — NOTES §6.1.1 budgets that bias.

- **Node:** `localizer_node` — Terminal 4, `ros2 launch tb3_bringup localizer.launch.py`
- **Config:** [`config/localizer.yaml`](config/localizer.yaml) — `scan_window_half` and the range gates interlock with the detector thresholds; read NOTES §2 before changing any of them
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
