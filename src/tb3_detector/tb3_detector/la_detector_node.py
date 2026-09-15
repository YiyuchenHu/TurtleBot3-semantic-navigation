#!/usr/bin/env python3
"""
la_detector_node.py — LocateAnything mapping-path detector (Phase 2b).

Drop-in replacement producer for the YOLO detector_node: subscribes to
the camera, calls the grounding server's fixed-vocabulary POST /detect
over HTTP, and publishes vision_msgs/Detection2DArray on the SAME
absolute topic (/detector_node/detections) with header.stamp set to the
SOURCE IMAGE stamp — so the downstream localizer → memory → map-memory →
query chain runs unchanged (R01 §5.1/§5.3).

Differences from detector_node.py, all deliberate:
  * No local model: inference is one HTTP round-trip (~0.2–0.3 s) to the
    grounding container sharing this container's network namespace.
  * Frame sampling: at most one request per `min_interval_sec` of ROS
    time, and frames are skipped while |angular velocity| (from /odom)
    exceeds `max_angular_vel` — the HTTP latency plus the localizer's
    latest-scan bearing match punish detections taken mid-spin.
  * Cross-class dedup: the model may list one object under several
    vocabulary words (R01 §2.3); overlapping boxes of different labels
    (IoU ≥ `cross_class_iou`) collapse to the earliest vocabulary entry.

The YOLO node stays in the package untouched; only the launch decides
which producer runs (ground rule 7).
"""

from __future__ import annotations

import base64
import json
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import cv2

from cv_bridge import CvBridge


def iou_xyxy(a: list[float], b: list[float]) -> float:
    """Plain IoU of two [x1, y1, x2, y2] boxes."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / max(area_a + area_b - inter, 1e-9)


def dedup_cross_class(dets: list[dict], priority: dict[str, int],
                      iou_threshold: float) -> tuple[list[dict], int]:
    """Collapse overlapping boxes of DIFFERENT labels onto the label with
    the best (lowest) vocabulary priority. Same-label overlaps are left
    alone — genuinely distinct instances of one class may overlap.

    Returns (kept, n_dropped)."""
    order = sorted(dets, key=lambda d: priority.get(d["label"], len(priority)))
    kept: list[dict] = []
    dropped = 0
    for d in order:
        clash = any(
            k["label"] != d["label"]
            and iou_xyxy(k["bbox_xyxy"], d["bbox_xyxy"]) >= iou_threshold
            for k in kept
        )
        if clash:
            dropped += 1
        else:
            kept.append(d)
    return kept, dropped


class LaDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__("la_detector_node")

        self.declare_parameter("server_url", "http://127.0.0.1:8801")
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("vocabulary",
                               ["person", "bookshelf", "trash can", "traffic cone"])
        self.declare_parameter("min_interval_sec", 0.3)
        self.declare_parameter("max_angular_vel", 0.5)
        self.declare_parameter("cross_class_iou", 0.8)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("detections_topic", "/detector_node/detections")
        self.declare_parameter("debug_image_topic", "/detector_node/debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("stats_interval_sec", 10.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._server_url = str(g("server_url")).rstrip("/")
        self._timeout = float(g("request_timeout_sec"))
        self._vocabulary = [str(v) for v in g("vocabulary") if str(v).strip()]
        self._priority = {v: i for i, v in enumerate(self._vocabulary)}
        self._min_interval = float(g("min_interval_sec"))
        self._max_ang_vel = float(g("max_angular_vel"))
        self._cross_iou = float(g("cross_class_iou"))
        self._pub_debug = bool(g("publish_debug_image"))
        self._jpeg_quality = int(g("jpeg_quality"))
        queue = int(g("queue_size"))
        self._stats_interval = float(g("stats_interval_sec"))

        if not self._vocabulary:
            raise ValueError("la_detector_node: vocabulary must not be empty")

        self._bridge = CvBridge()
        self._last_request_t: float | None = None   # ROS time, seconds
        self._angular_vel = 0.0
        self._server_seen = False

        # Rolling stats for the [la_stats] line (report metrics: per-frame
        # /detect latency and cross-class dedup count).
        self._st = dict(frames=0, skipped_interval=0, skipped_spin=0,
                        requests=0, errors=0, dedup_dropped=0,
                        lat_sum=0.0, lat_max=0.0, det_counts={})
        self._last_stats_wall = time.monotonic()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=queue)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE, depth=queue)

        self.create_subscription(Image, str(g("image_topic")),
                                 self._image_cb, sensor_qos)
        self.create_subscription(Odometry, str(g("odom_topic")),
                                 self._odom_cb, sensor_qos)
        self._det_pub = self.create_publisher(
            Detection2DArray, str(g("detections_topic")), reliable_qos)
        self._dbg_pub = (self.create_publisher(
            Image, str(g("debug_image_topic")), reliable_qos)
            if self._pub_debug else None)

        self._check_server_health()
        self.get_logger().info(
            "la_detector_node ready. server=%s vocabulary=%s "
            "min_interval=%.2fs max_ang_vel=%.2frad/s cross_iou=%.2f"
            % (self._server_url, self._vocabulary, self._min_interval,
               self._max_ang_vel, self._cross_iou))

    # ── Callbacks ───────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._angular_vel = msg.twist.twist.angular.z

    def _image_cb(self, msg: Image) -> None:
        self._st["frames"] += 1
        now = self.get_clock().now().nanoseconds * 1e-9

        if (self._last_request_t is not None
                and now - self._last_request_t < self._min_interval):
            self._st["skipped_interval"] += 1
            return
        if abs(self._angular_vel) > self._max_ang_vel:
            self._st["skipped_spin"] += 1
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning("cv_bridge conversion failed: %s" % e)
            return

        # Mark the slot BEFORE the blocking HTTP call: queued frames that
        # drain right after it returns must not fire immediately again.
        self._last_request_t = now

        t0 = time.monotonic()
        result = self._post_detect(bgr)
        rtt_ms = (time.monotonic() - t0) * 1000.0
        if result is None:
            return
        self._st["requests"] += 1
        self._st["lat_sum"] += rtt_ms
        self._st["lat_max"] = max(self._st["lat_max"], rtt_ms)

        dets = [d for d in result.get("detections", [])
                if d.get("label") in self._priority]
        dets, dropped = dedup_cross_class(dets, self._priority, self._cross_iou)
        self._st["dedup_dropped"] += dropped
        for d in dets:
            self._st["det_counts"][d["label"]] = \
                self._st["det_counts"].get(d["label"], 0) + 1

        self._det_pub.publish(self._to_msg(dets, msg.header))
        if self._dbg_pub is not None:
            self._publish_debug(bgr, dets, rtt_ms, msg.header)
        self._maybe_log_stats()

    # ── HTTP ────────────────────────────────────────────────────────────

    def _post_detect(self, bgr) -> dict | None:
        ok, buf = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            self.get_logger().warning("JPEG encode failed, skipping frame")
            return None
        payload = json.dumps({
            "image": base64.b64encode(buf.tobytes()).decode("ascii"),
            "categories": self._vocabulary,
        }).encode()
        req = urllib.request.Request(
            self._server_url + "/detect", data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                out = json.load(resp)
            if not self._server_seen:
                self._server_seen = True
                self.get_logger().info(
                    "grounding server responding (backend=%s)"
                    % out.get("backend"))
            return out
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError,
                ConnectionError, OSError) as e:
            self._st["errors"] += 1
            # Throttled: during grounding cold start (~76 s) this would
            # otherwise flood the log at camera rate.
            self.get_logger().warning(
                "/detect failed: %s" % e, throttle_duration_sec=5.0)
            return None

    def _check_server_health(self) -> None:
        try:
            with urllib.request.urlopen(self._server_url + "/health",
                                        timeout=2.0) as resp:
                info = json.load(resp)
            self._server_seen = True
            self.get_logger().info("grounding /health: %s" % info)
            if info.get("backend") == "mock":
                self.get_logger().warning(
                    "grounding backend is MOCK — vocabulary words without "
                    "color terms will never be detected")
        except Exception as e:
            self.get_logger().warning(
                "grounding server not reachable yet (%s) — will keep "
                "trying per frame (cold start is ~76 s)" % e)

    # ── Publishing ──────────────────────────────────────────────────────

    def _to_msg(self, dets: list[dict], header) -> Detection2DArray:
        msg = Detection2DArray()
        # Source image stamp, NOT receive/response time: downstream TF and
        # scan association must see the same timeline as the YOLO node
        # produced (R01 §5.3).
        msg.header = header
        for d in dets:
            det = Detection2D()
            det.header = header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d["label"]
            # The model emits no confidence; server score is a constant.
            hyp.hypothesis.score = float(d.get("score", 1.0))
            det.results.append(hyp)
            x1, y1, x2, y2 = d["bbox_xyxy"]
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            msg.detections.append(det)
        return msg

    def _publish_debug(self, bgr, dets: list[dict], rtt_ms: float,
                       header) -> None:
        vis = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 120, 0), 2)
            cv2.putText(vis, d["label"], (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 0), 2)
        cv2.putText(vis, "LA /detect %.0f ms" % rtt_ms, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        try:
            dbg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            dbg.header = header
            self._dbg_pub.publish(dbg)
        except Exception as e:
            self.get_logger().warning("debug image publish failed: %s" % e)

    def _maybe_log_stats(self) -> None:
        if self._stats_interval <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_stats_wall < self._stats_interval:
            return
        self._last_stats_wall = now
        s = self._st
        avg = s["lat_sum"] / s["requests"] if s["requests"] else 0.0
        self.get_logger().info(
            "[la_stats] frames=%d requests=%d errors=%d "
            "skipped(interval=%d, spin=%d) latency(avg=%.0fms, max=%.0fms) "
            "dedup_dropped=%d detections=%s"
            % (s["frames"], s["requests"], s["errors"], s["skipped_interval"],
               s["skipped_spin"], avg, s["lat_max"], s["dedup_dropped"],
               s["det_counts"]))


def main(args=None):
    rclpy.init(args=args)
    node = LaDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
