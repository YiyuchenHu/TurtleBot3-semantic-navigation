# tb3_detector/models

YOLO26 weight files live here.

## Shipped with the repository

| Filename              | What it is |
|-----------------------|------------|
| `tb3det_yolo26n.pt`   | yolo26n fine-tuned on 9600 Gazebo frames of the three course targets (`person`, `trash_can`, `chair`). The detector the assignment runs. 5.4 MB, committed — see NOTES.md "How the detector was trained". |

After a fresh clone, `colcon build --symlink-install --packages-select tb3_detector`
copies it into the install tree; nothing to download.

## Optional (local download, git-ignored)

| Filename       | Download |
|----------------|----------|
| `yolo26n.pt`   | `wget -O src/tb3_detector/models/yolo26n.pt https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt` |

The COCO-80 weights the course used until 2026-09-03. Kept as the comparison
experiment in NOTES.md: point `model_path` at it and set
`class_filter: ["person", "traffic light", "chair"]` (COCO has no trash-can
class and calls the can a traffic light).

## .gitignore

`*.pt` is ignored here, with a single exception for `tb3det_yolo26n.pt`. Any
other weights you experiment with stay local.
