#!/usr/bin/env python3
"""Grab camera frames from the running sim stack (probe/debug utility).

The tool every perception probe starts with — capture what the robot actually
sees, then feed the frames to grounding/eval scripts or POST /detect.

Run INSIDE the sim container while the stack is up:

    source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
    python3 ~/ws/scripts/grab_frames.py --out /tmp/capture -n 12 -p 4.0

Saves frame_NN.jpg + capture_index.json into --out (default ./capture).
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, out: Path, n_frames: int, period_s: float, topic: str):
        super().__init__("frame_grabber")
        self.out = out
        self.n_frames = n_frames
        self.period_s = period_s
        self.bridge = CvBridge()
        self.latest = None
        self.saved = 0
        self.index = {}
        self.next_t = time.monotonic() + 2.0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self.on_img, qos)
        self.create_timer(0.2, self.tick)

    def on_img(self, msg):
        self.latest = msg

    def tick(self):
        if self.latest is None or time.monotonic() < self.next_t:
            return
        self.next_t = time.monotonic() + self.period_s
        name = "frame_%02d.jpg" % self.saved
        bgr = self.bridge.imgmsg_to_cv2(self.latest, desired_encoding="bgr8")
        cv2.imwrite(str(self.out / name), bgr,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        self.index[name] = {"t_wall": time.time()}
        self.saved += 1
        print("saved %s" % name, flush=True)
        if self.saved >= self.n_frames:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="capture", help="output directory")
    ap.add_argument("-n", "--frames", type=int, default=45,
                    help="number of frames to save")
    ap.add_argument("-p", "--period", type=float, default=8.0,
                    help="seconds between saved frames")
    ap.add_argument("--topic", default="/camera/image_raw")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Grabber(out, args.frames, args.period, args.topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    (out / "capture_index.json").write_text(json.dumps(node.index, indent=2))
    print("done: %d frames -> %s" % (node.saved, out))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
