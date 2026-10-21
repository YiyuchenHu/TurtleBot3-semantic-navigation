#!/usr/bin/env python3
"""
semantic_runtime_debug_node.py — runtime diagnostics for the three live targets.

Subscribes to the detector, localizer and memory topics without modifying any
existing node.  Every `summary_interval_sec` it prints one block that walks a
target through the whole pipeline, so you can see WHICH STAGE is losing it:

    detect  ->  localize  ->  remember

That is the question this node exists to answer.  "The chair never shows up in
RViz" has three completely different fixes depending on whether YOLO never saw
it (conf_threshold / viewing angle), the localizer saw it but could not attach
a LiDAR range (scan_window_half / range gates), or memory saw it but never
promoted it (chair_min_observations).

Counting is done by DETECTOR LABEL, because that is what every topic in the
chain carries.  With the fine-tuned weights the three labels equal the semantic
names; the table still prints both so it keeps working with COCO weights, where
the trash can is reported as "traffic light".

Launch it alongside the backend:
    ros2 launch tb3_bringup backend.launch.py use_runtime_debug:=true
    ros2 launch tb3_bringup full_stack.launch.py use_runtime_debug:=true
"""

import csv
import math
import os
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray


class SemanticRuntimeDebugNode(Node):

    # Detector labels of the three enabled targets in
    # tb3_bringup/config/semantic_targets.yaml, in report order.
    FOCUS_CLASSES = ("person", "trash_can", "chair")

    # detector_label -> semantic_name, so the table can print both.  The COCO
    # alias is kept so the node still counts correctly if someone runs the
    # optional COCO yolo26n comparison (NOTES.md), where the trash can is
    # reported as "traffic light".
    SEMANTIC_NAME = {
        "person": "person",
        "trash_can": "trash_can",
        "traffic light": "trash_can",
        "chair": "chair",
    }

    # The one pair the COCO weights actually confused: a partially clipped
    # chair and a partially clipped trash can swapped labels (coordinator.yaml
    # raised both promotion thresholds to 12 because of exactly this).  Frames
    # where both labels land on the SAME pixels are the observable signature.
    SWAP_PAIR = ("chair", "trash_can")

    def __init__(self):
        super().__init__("semantic_runtime_debug_node")

        self.declare_parameter("summary_interval_sec", 5.0)
        self.declare_parameter("csv_log_enabled", True)
        self.declare_parameter("csv_log_dir", "/tmp/semantic_debug")
        self.declare_parameter("detections_topic", "/detector_node/detections")
        self.declare_parameter("debug_image_topic", "/detector_node/debug_image")
        self.declare_parameter("localized_topic", "/localizer_node/localized_objects")
        self.declare_parameter("memory_topic", "/semantic_memory_node/objects")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        # Two boxes of different classes overlapping by more than this are
        # treated as one object claimed by two labels.
        self.declare_parameter("swap_iou_threshold", 0.5)

        det_topic = self.get_parameter("detections_topic").value
        dbg_topic = self.get_parameter("debug_image_topic").value
        loc_topic = self.get_parameter("localized_topic").value
        mem_topic = self.get_parameter("memory_topic").value
        cam_topic = self.get_parameter("camera_topic").value
        self._swap_iou = float(self.get_parameter("swap_iou_threshold").value)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            Detection2DArray, det_topic, self._cb_detections, reliable_qos)
        self.create_subscription(
            Image, dbg_topic, self._cb_debug_image, best_effort_qos)
        self.create_subscription(
            Detection3DArray, loc_topic, self._cb_localized, reliable_qos)
        self.create_subscription(
            Detection3DArray, mem_topic, self._cb_memory, reliable_qos)
        self.create_subscription(
            Image, cam_topic, self._cb_camera, best_effort_qos)

        self._det_count = defaultdict(int)
        self._det_conf_sum = defaultdict(float)
        self._det_conf_min = {}
        self._loc_count = defaultdict(int)
        self._loc_range_sum = defaultdict(float)
        self._mem_active = defaultdict(int)
        self._swap_frames = 0

        self._cam_times = []
        self._det_times = []
        self._dbg_times = []
        self._loc_times = []

        self._summary_n = 0
        self._start_wall = time.monotonic()

        interval = self.get_parameter("summary_interval_sec").value
        self.create_timer(interval, self._print_summary)

        self._csv_writer = None
        self._csv_file = None
        if self.get_parameter("csv_log_enabled").value:
            log_dir = self.get_parameter("csv_log_dir").value
            try:
                Path(log_dir).mkdir(parents=True, exist_ok=True)
                ts = time.strftime("%Y%m%d_%H%M%S")
                csv_path = os.path.join(log_dir, f"semantic_debug_{ts}.csv")
                self._csv_file = open(csv_path, "w", newline="")
            except OSError as exc:
                # A read-only or missing log dir must not take the node down:
                # this is an opt-in diagnostic, not part of the pipeline.
                self.get_logger().warning(
                    f"CSV logging disabled, could not open {log_dir}: {exc}")
            else:
                self._csv_writer = csv.writer(self._csv_file)
                self._csv_writer.writerow([
                    "wall_time", "source", "class", "confidence",
                    "bbox_cx", "bbox_cy", "bbox_w", "bbox_h",
                    "x", "y", "range_m", "bearing_deg",
                ])
                self.get_logger().info(f"CSV log -> {csv_path}")

    def destroy_node(self):
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()

    # -- Callbacks --

    def _cb_camera(self, msg: Image):
        self._cam_times.append(time.monotonic())
        self._trim(self._cam_times)

    def _cb_debug_image(self, msg: Image):
        self._dbg_times.append(time.monotonic())
        self._trim(self._dbg_times)

    def _cb_detections(self, msg: Detection2DArray):
        now = time.monotonic()
        self._det_times.append(now)
        self._trim(self._det_times)

        boxes_this_frame = []
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            label = best.hypothesis.class_id
            conf = best.hypothesis.score

            self._det_count[label] += 1
            self._det_conf_sum[label] += conf
            prev = self._det_conf_min.get(label)
            self._det_conf_min[label] = conf if prev is None else min(prev, conf)

            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            boxes_this_frame.append((label, cx, cy, bw, bh))

            if self._csv_writer:
                self._csv_writer.writerow([
                    f"{now:.3f}", "det", label, f"{conf:.3f}",
                    f"{cx:.1f}", f"{cy:.1f}", f"{bw:.1f}", f"{bh:.1f}",
                    "", "", "", "",
                ])

        self._check_label_swap(boxes_this_frame)

    def _cb_localized(self, msg: Detection3DArray):
        now = time.monotonic()
        self._loc_times.append(now)
        self._trim(self._loc_times)

        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            label = best.hypothesis.class_id
            conf = best.hypothesis.score
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            rng = math.hypot(x, y)
            bearing = math.degrees(math.atan2(-y, x)) if rng > 0.01 else 0.0

            self._loc_count[label] += 1
            self._loc_range_sum[label] += rng

            if self._csv_writer:
                self._csv_writer.writerow([
                    f"{now:.3f}", "loc", label, f"{conf:.3f}",
                    "", "", "", "",
                    f"{x:.3f}", f"{y:.3f}", f"{rng:.3f}", f"{bearing:.1f}",
                ])

    def _cb_memory(self, msg: Detection3DArray):
        snapshot = defaultdict(int)
        for det in msg.detections:
            if not det.results:
                continue
            label = max(det.results, key=lambda r: r.hypothesis.score).hypothesis.class_id
            snapshot[label] += 1
        self._mem_active = snapshot

    # -- Label-swap analysis --

    @staticmethod
    def _iou(a, b):
        """IoU of two (cx, cy, w, h) boxes in pixels."""
        _, acx, acy, aw, ah = a
        _, bcx, bcy, bw, bh = b
        ax0, ax1 = acx - aw / 2.0, acx + aw / 2.0
        ay0, ay1 = acy - ah / 2.0, acy + ah / 2.0
        bx0, bx1 = bcx - bw / 2.0, bcx + bw / 2.0
        by0, by1 = bcy - bh / 2.0, bcy + bh / 2.0
        ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        iy = max(0.0, min(ay1, by1) - max(ay0, by0))
        inter = ix * iy
        union = aw * ah + bw * bh - inter
        return inter / union if union > 0 else 0.0

    def _check_label_swap(self, boxes):
        """Count frames where the confusable pair claims the same pixels.

        Unlike a bare co-occurrence count, this cannot fire just because both
        objects are legitimately in view: it needs two boxes of DIFFERENT
        classes sitting on top of each other, which is what a mislabel of one
        physical object looks like.
        """
        a_boxes = [b for b in boxes if b[0] == self.SWAP_PAIR[0]]
        b_boxes = [b for b in boxes if b[0] == self.SWAP_PAIR[1]]
        for a in a_boxes:
            for b in b_boxes:
                if self._iou(a, b) >= self._swap_iou:
                    self._swap_frames += 1
                    return

    # -- Periodic summary --

    def _print_summary(self):
        self._summary_n += 1
        elapsed = time.monotonic() - self._start_wall

        cam_fps = self._fps(self._cam_times)
        det_fps = self._fps(self._det_times)
        dbg_fps = self._fps(self._dbg_times)
        loc_fps = self._fps(self._loc_times)

        det_interval = self._avg_interval(self._det_times)
        dbg_interval = self._avg_interval(self._dbg_times)
        dbg_lag = ""
        if det_interval > 0 and dbg_interval > 0:
            ratio = dbg_interval / det_interval
            if ratio > 2.0:
                dbg_lag = f" WARNING debug_image {ratio:.1f}x slower than detections"

        lines = [
            f"=== Semantic Debug #{self._summary_n}  t={elapsed:.0f}s ===",
            f"  FPS  cam={cam_fps:.1f}  det={det_fps:.1f}  dbg_img={dbg_fps:.1f}  loc={loc_fps:.1f}{dbg_lag}",
            "  target      label           detect  avg_conf min_conf  localize  avg_m  memory",
        ]

        for cls in self.FOCUS_CLASSES:
            n = self._det_count.get(cls, 0)
            avg_c = (self._det_conf_sum[cls] / n) if n > 0 else 0.0
            min_c = self._det_conf_min.get(cls, 0.0)
            n_loc = self._loc_count.get(cls, 0)
            avg_r = (self._loc_range_sum[cls] / n_loc) if n_loc > 0 else 0.0
            n_mem = self._mem_active.get(cls, 0)
            lines.append(
                f"  {self.SEMANTIC_NAME[cls]:11s} {cls:15s} {n:6d}  {avg_c:8.2f} {min_c:8.2f}  "
                f"{n_loc:8d} {avg_r:6.2f}  {n_mem:6d}"
            )

        other = {c: n for c, n in self._det_count.items() if c not in self.FOCUS_CLASSES}
        if other:
            lines.append("  off-target detections (class_filter should exclude these): "
                         + ", ".join(f"{c}={n}" for c, n in sorted(other.items())))

        # Per-stage diagnosis: name the stage that is losing each target.
        #
        # A stage that is not running at all must not be reported as a stage
        # that is running badly. Running only T5 (detector) against
        # detector_test.world is a normal thing to do, and blaming
        # localizer.yaml for it would send a student to the wrong file.
        localizer_silent = loc_fps == 0.0 and sum(self._loc_count.values()) == 0
        memory_silent = not self._mem_active
        if localizer_silent:
            lines.append("  [localize] no messages on the localizer topic at all — "
                         "is Terminal 4 (localizer.launch.py) running?")
        for cls in self.FOCUS_CLASSES:
            name = self.SEMANTIC_NAME[cls]
            n, n_loc = self._det_count.get(cls, 0), self._loc_count.get(cls, 0)
            n_mem = self._mem_active.get(cls, 0)
            if n == 0:
                lines.append(f"  [detect]   {name}: not seen yet — check viewing angle, "
                             f"distance, and detector.yaml conf_threshold/class_filter")
            elif n_loc == 0 and not localizer_silent:
                lines.append(f"  [localize] {name}: detected {n}x but never localized — LiDAR "
                             f"association failing (localizer.yaml scan_window_half / "
                             f"min_valid_range / max_valid_range)")
            elif n_loc > 0 and n_mem == 0 and not memory_silent:
                lines.append(f"  [remember] {name}: localized {n_loc}x but not in memory yet — "
                             f"below its promotion threshold in coordinator.yaml")

        if self._swap_frames:
            lines.append(
                f"  LABEL SWAP: {self._swap_frames} frame(s) where "
                f"'{self.SWAP_PAIR[0]}' and '{self.SWAP_PAIR[1]}' boxes overlapped "
                f"(IoU >= {self._swap_iou:.2f}) — one object claimed by both labels")

        self.get_logger().info("\n".join(lines))

        if self._csv_file:
            self._csv_file.flush()

    # -- Helpers --

    @staticmethod
    def _trim(timestamps, window=10.0):
        cutoff = time.monotonic() - window
        while timestamps and timestamps[0] < cutoff:
            timestamps.pop(0)

    @staticmethod
    def _fps(timestamps):
        if len(timestamps) < 2:
            return 0.0
        span = timestamps[-1] - timestamps[0]
        return (len(timestamps) - 1) / span if span > 0 else 0.0

    @staticmethod
    def _avg_interval(timestamps):
        if len(timestamps) < 2:
            return 0.0
        intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
        return sum(intervals) / len(intervals)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRuntimeDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
