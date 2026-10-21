# `tb3_detector`

Stage 1: YOLO inference on `/camera/image_raw`, publishing `vision_msgs/Detection2DArray` on `/detector_node/detections` plus an annotated `/detector_node/debug_image`. Labels are the detector's own class strings (`person`, `trash_can`, `chair`) — the rest of the stack translates them via `tb3_bringup/config/semantic_targets.yaml`.

- **Node:** `detector_node` — Terminal 5, `ros2 launch tb3_bringup detector.launch.py`
- **Config:** [`config/detector.yaml`](config/detector.yaml) (`conf_threshold`, `class_filter` — detector labels, never semantic names). Weights: `models/tb3det_yolo26n.pt` ships with the repo (fine-tuned on simulator frames, see the [README](../../README.md))
- **Docs:** [README](../../README.md)
