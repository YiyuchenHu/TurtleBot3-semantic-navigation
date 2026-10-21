#!/usr/bin/env python3
"""In-container helper: capture camera frames for the grounding eval.

Runs INSIDE the sim container (needs rclpy + cv2; both imports are
guarded with a clear error if you run it on the host by mistake).

Workflow per viewpoint: drive or spawn the robot there, then

    python3 grounding/eval/capture_frames.py --grab chairs_front

which waits for the next /camera/image_raw frame, writes
grounding/eval/captures/chairs_front.jpg and prints the path.

    --poses viewpoints.example.json   list the named viewpoints
    --annotate captures/foo.jpg       print manifest-labeling help for a
                                      saved frame (no GUI); if
                                      `ultralytics` is importable, also
                                      runs YOLO once and prints bbox
                                      suggestions as paste-ready JSON
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

_EVAL_DIR = pathlib.Path(__file__).resolve().parent
_DEFAULT_OUT = _EVAL_DIR / "captures"
_DEFAULT_TOPIC = "/camera/image_raw"


def _die(msg: str) -> "None":
    sys.exit(f"error: {msg}")


# --------------------------------------------------------------------------
# --grab
# --------------------------------------------------------------------------

def grab(name: str, topic: str, out_dir: pathlib.Path, timeout: float) -> None:
    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError as exc:
        _die(
            f"missing dependency ({exc}). --grab must run inside the sim "
            "container where rclpy and cv2 are installed, e.g.\n"
            "  docker compose exec sim python3 "
            "grounding/eval/capture_frames.py --grab " + name
        )

    def to_bgr(msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("bgr8", "rgb8"):
            img = buf.reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img
        if msg.encoding == "mono8":
            return cv2.cvtColor(buf.reshape(msg.height, msg.width),
                                cv2.COLOR_GRAY2BGR)
        _die(f"unsupported image encoding '{msg.encoding}' on {topic}")

    rclpy.init()
    node = Node("eval_frame_grabber")
    frame = {}

    def cb(msg):
        frame["img"] = to_bgr(msg)

    node.create_subscription(Image, topic, cb, 1)
    node.get_logger().info(f"waiting for one frame on {topic} ...")
    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "img" not in frame:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds / 1e9 > deadline:
            node.destroy_node()
            rclpy.shutdown()
            _die(
                f"no frame on {topic} within {timeout:.0f}s — is the sim "
                "running and the camera publishing? "
                "(ros2 topic hz " + topic + ")"
            )
    node.destroy_node()
    rclpy.shutdown()

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{name}.jpg"
    if not cv2.imwrite(str(out_path), frame["img"]):
        _die(f"cv2.imwrite failed for {out_path}")
    print(out_path)


# --------------------------------------------------------------------------
# --poses
# --------------------------------------------------------------------------

def show_poses(path: pathlib.Path) -> None:
    if not path.is_file():
        _die(f"poses file not found: {path}")
    text = path.read_text()
    try:
        poses = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
            poses = yaml.safe_load(text)
        except ImportError:
            _die(f"{path} is not JSON and PyYAML is not installed")
    if not isinstance(poses, dict):
        _die(f"{path} must map viewpoint name -> pose")

    print(f"viewpoints in {path}:")
    print("(drive or spawn the robot there, then run with --grab <name>)\n")
    for name, pose in poses.items():
        desc = pose.pop("description", "") if isinstance(pose, dict) else ""
        print(f"  {name:<16} {json.dumps(pose)}  {desc}")
    print(
        "\nexample teleport (Gazebo Harmonic):\n"
        "  gz service -s /world/<world>/set_pose --reqtype gz.msgs.Pose \\\n"
        "    --reptype gz.msgs.Boolean --timeout 2000 \\\n"
        "    --req 'name: \"waffle_pi\", position: {x: -3, y: 0, z: 0.01}'\n"
        "(the world name is the <world name=...> in the .world file)\n"
        "or use teleop and eyeball it, then:\n"
        "  python3 grounding/eval/capture_frames.py --grab <name>"
    )


# --------------------------------------------------------------------------
# --annotate
# --------------------------------------------------------------------------

def annotate(frame_path: pathlib.Path) -> None:
    if not frame_path.is_file():
        _die(f"frame not found: {frame_path} — capture it first with --grab")
    try:
        import cv2
    except ImportError as exc:
        _die(
            f"missing dependency ({exc}). --annotate needs cv2; run it "
            "inside the sim container."
        )

    img = cv2.imread(str(frame_path))
    if img is None:
        _die(f"could not decode {frame_path}")
    h, w = img.shape[:2]

    print(f"frame: {frame_path}  ({w}x{h} px)")
    print(
        "\nTo label ground truth by hand:\n"
        "  1. open the image in any viewer that shows pixel coordinates\n"
        "     (e.g. `python3 -m http.server` in the captures dir and use\n"
        "     the browser, or GIMP/Preview pointer readout)\n"
        "  2. note the object's top-left (x1, y1) and bottom-right\n"
        "     (x2, y2) corners\n"
        "  3. paste into the manifest case, replacing the placeholder:\n"
        '       "gt_bbox_xyxy": [x1, y1, x2, y2]\n'
        "     distractor boxes go in \"distractor_bboxes\": [[...], ...]\n"
    )

    try:
        from ultralytics import YOLO
    except ImportError:
        print("(ultralytics not installed — skipping YOLO bbox "
              "suggestions; labels above are manual-only)")
        return

    print("YOLO suggestions (verify before pasting — COCO classes only,")
    print("so 'pallet jack' etc. will be missing or mislabeled):\n")
    results = YOLO("yolov8n.pt")(img, verbose=False)
    suggestions = []
    for r in results:
        for b in r.boxes:
            x1, y1, x2, y2 = (round(float(v), 1) for v in b.xyxy[0])
            suggestions.append({
                "label": r.names[int(b.cls[0])],
                "conf": round(float(b.conf[0]), 3),
                "gt_bbox_xyxy": [x1, y1, x2, y2],
            })
    if not suggestions:
        print("  (no detections)")
    for s in suggestions:
        print(f"  # {s['label']} (conf {s['conf']})")
        print(f'  "gt_bbox_xyxy": {json.dumps(s["gt_bbox_xyxy"])}')


# --------------------------------------------------------------------------

def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description="Capture / annotate eval frames (run in-container)")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--grab", metavar="NAME",
                      help="wait for the next camera frame and save "
                           "captures/NAME.jpg")
    mode.add_argument("--poses", metavar="FILE", type=pathlib.Path,
                      help="print the named robot viewpoints in FILE "
                           "(JSON or YAML)")
    mode.add_argument("--annotate", metavar="FRAME", type=pathlib.Path,
                      help="print manifest-labeling instructions (and "
                           "YOLO bbox suggestions if available) for a "
                           "saved frame")
    ap.add_argument("--topic", default=_DEFAULT_TOPIC)
    ap.add_argument("--out-dir", type=pathlib.Path, default=_DEFAULT_OUT)
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for a frame (--grab)")
    args = ap.parse_args(argv)

    if args.grab:
        grab(args.grab, args.topic, args.out_dir, args.timeout)
    elif args.poses:
        show_poses(args.poses)
    else:
        annotate(args.annotate)


if __name__ == "__main__":
    main()
