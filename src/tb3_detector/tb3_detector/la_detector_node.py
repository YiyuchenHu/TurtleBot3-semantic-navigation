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
from std_msgs.msg import String
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


def filter_bearing_occluded(dets: list[dict],
                            min_bottom_gap_px: float = 8.0
                            ) -> tuple[list[dict], int]:
    """Drop detections the planar localizer is guaranteed to mislocate.

    The localizer turns a bbox into (x, y) by casting the scan ray at the
    bbox CENTRE column — so when a farther object D is seen past a nearer
    object E (their image columns overlap), the ray hits E and D's
    observation lands on E's position. Observed live in run 1: the cone
    behind the centre person (seen from the west) and the bookshelf
    behind the same person (seen from the south) each planted hundreds of
    wrong-label observations on the person, promoting fake landmarks.

    Rule: drop D if some other-class detection E spans D's centre column
    and E's bottom edge is at least `min_bottom_gap_px` lower (both
    classes stand on the floor, so a lower bottom edge means nearer).
    """
    kept: list[dict] = []
    dropped = 0
    for d in dets:
        x1, _y1, x2, y2 = d["bbox_xyxy"]
        u = (x1 + x2) / 2.0
        occluded = any(
            e is not d and e["label"] != d["label"]
            and e["bbox_xyxy"][0] <= u <= e["bbox_xyxy"][2]
            and e["bbox_xyxy"][3] - y2 >= min_bottom_gap_px
            for e in dets
        )
        if occluded:
            dropped += 1
        else:
            kept.append(d)
    return kept, dropped


def labels_agree(a: str, b: str) -> bool:
    """True when one label is a token-subset of the other — the two names
    describe the SAME object family, not a forced-choice conflict.

    "person" ⊆ "person wearing a green shirt" → agree (the model saying
    both about one box is double-labeling, not a mislabel). "person
    wearing a red shirt" vs "person wearing a green shirt" differ in a
    token → conflict. Used by the coverage veto (R04): without it, the
    shirt-color coverage categories veto the very observation a person
    search (or a color search) is trying to accumulate.
    """
    ta, tb = set(a.split()), set(b.split())
    return ta <= tb or tb <= ta


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
        self.declare_parameter("occlusion_filter", True)
        self.declare_parameter("active_search_topic", "/active_search")
        # Coverage categories: requested from the model and used for the
        # occlusion filter and same-object veto, but NEVER published —
        # they keep the geometric safety nets sighted without putting
        # the classes into the mapping vocabulary (R02c smoke run 1
        # post-mortem: with vocabulary ["person"] alone, both fake
        # landmarks were caused by boxes the filters could no longer
        # see).
        self.declare_parameter("coverage_categories",
                               ["bookshelf", "trash can", "traffic cone"])
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
        self._base_vocabulary = [str(v) for v in g("vocabulary") if str(v).strip()]
        # ── Search-link injection (Phase 2c) ────────────────────────────
        # /active_search carries one free-form expression (empty = none).
        # While active it is PREPENDED to the vocabulary, so it holds the
        # highest cross-class dedup priority — an overlapping box keeps
        # the search label, never a base-vocabulary one.
        self._search_term: str | None = None
        self._coverage_base = [str(v).lower() for v in g("coverage_categories")
                               if str(v).strip()]
        self._vocabulary: list[str] = []
        self._priority: dict[str, int] = {}
        self._coverage: list[str] = []
        self._rebuild_vocabulary()
        self._min_interval = float(g("min_interval_sec"))
        self._max_ang_vel = float(g("max_angular_vel"))
        self._cross_iou = float(g("cross_class_iou"))
        self._occlusion_filter = bool(g("occlusion_filter"))
        self._pub_debug = bool(g("publish_debug_image"))
        self._jpeg_quality = int(g("jpeg_quality"))
        queue = int(g("queue_size"))
        self._stats_interval = float(g("stats_interval_sec"))

        # R04 search-first profile: an EMPTY base vocabulary is allowed as
        # long as coverage categories exist — nothing is published (and
        # nothing enters memory) between searches; only the /active_search
        # expression is ever published. Both empty would send empty
        # category lists to /detect, which stays an error.
        if not self._base_vocabulary and not self._coverage_base:
            raise ValueError(
                "la_detector_node: vocabulary and coverage_categories "
                "must not both be empty")

        self._bridge = CvBridge()
        self._last_request_t: float | None = None   # ROS time, seconds
        self._angular_vel = 0.0
        self._server_seen = False

        # Rolling stats for the [la_stats] line (report metrics: per-frame
        # /detect latency and cross-class dedup count).
        self._st = dict(frames=0, skipped_interval=0, skipped_spin=0,
                        requests=0, errors=0, dedup_dropped=0,
                        occl_dropped=0, cover_vetoed=0, lat_sum=0.0,
                        lat_max=0.0, det_counts={}, cover_counts={})
        self._last_stats_wall = time.monotonic()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=queue)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE, depth=queue)

        self.create_subscription(Image, str(g("image_topic")),
                                 self._image_cb, sensor_qos)
        self.create_subscription(Odometry, str(g("odom_topic")),
                                 self._odom_cb, sensor_qos)
        # Latched: a term published before this node (re)starts still
        # arrives. The coordinator is the only publisher.
        search_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                depth=1)
        self.create_subscription(String, str(g("active_search_topic")),
                                 self._search_cb, search_qos)
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

    # ── Search-link vocabulary injection ────────────────────────────────

    def _rebuild_vocabulary(self) -> None:
        """Recompute the effective vocabulary and dedup priorities.

        The search term (if any) goes FIRST: index 0 is the best (lowest)
        priority in dedup_cross_class, so a search-term box always wins a
        cross-class overlap against base-vocabulary boxes (Phase 2c
        constraint a). A term that equals a base word is not duplicated.
        """
        base = self._base_vocabulary
        if self._search_term and self._search_term not in base:
            self._vocabulary = [self._search_term] + base
        else:
            self._vocabulary = list(base)
        self._priority = {v: i for i, v in enumerate(self._vocabulary)}
        # Coverage = configured coverage minus anything that is being
        # published anyway (base vocab or the active search term — a
        # searched class must not veto itself).
        self._coverage = [c for c in self._coverage_base
                          if c not in self._priority]

    def _search_cb(self, msg: String) -> None:
        term = msg.data.strip().lower() or None
        if term == self._search_term:
            return
        old = self._search_term
        self._search_term = term
        self._rebuild_vocabulary()
        if term:
            self.get_logger().info(
                "[search] vocabulary injected: '%s' (top dedup priority) "
                "→ categories=%s" % (term, self._vocabulary))
        else:
            self.get_logger().info(
                "[search] vocabulary cleared (was '%s') → categories=%s"
                % (old, self._vocabulary))

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

        coverage = set(self._coverage)
        all_dets = [d for d in result.get("detections", [])
                    if d.get("label") in self._priority
                    or d.get("label") in coverage]
        # Occlusion filter runs over publishable AND coverage boxes: a
        # nearer coverage object (e.g. the undetectable-by-vocabulary
        # cone) still shields the scan column behind it.
        if self._occlusion_filter:
            all_dets, occl = filter_bearing_occluded(all_dets)
            self._st["occl_dropped"] += occl
        cover_dets = [d for d in all_dets if d["label"] in coverage]
        dets = [d for d in all_dets if d["label"] in self._priority]
        # Coverage veto: a publishable box that is the SAME object as a
        # coverage box (IoU >= cross_class_iou) is a forced-choice
        # mislabel — the model also named it by its coverage class.
        # R02c smoke run 1: the cone collecting "trash can" boxes this
        # way promoted a fake landmark the search then navigated to.
        if cover_dets:
            kept = []
            for d in dets:
                clash = any(iou_xyxy(c["bbox_xyxy"], d["bbox_xyxy"])
                            >= self._cross_iou
                            and not labels_agree(c["label"], d["label"])
                            for c in cover_dets)
                if clash:
                    self._st["cover_vetoed"] += 1
                else:
                    kept.append(d)
            dets = kept
        dets, dropped = dedup_cross_class(dets, self._priority, self._cross_iou)
        self._st["dedup_dropped"] += dropped
        for d in dets:
            self._st["det_counts"][d["label"]] = \
                self._st["det_counts"].get(d["label"], 0) + 1
        for d in cover_dets:
            self._st["cover_counts"][d["label"]] = \
                self._st["cover_counts"].get(d["label"], 0) + 1

        self._det_pub.publish(self._to_msg(dets, msg.header))
        if self._dbg_pub is not None:
            self._publish_debug(bgr, dets, rtt_ms, msg.header,
                                cover_dets=cover_dets)
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
            "categories": self._vocabulary + self._coverage,
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
                       header, cover_dets: list[dict] | None = None) -> None:
        vis = bgr.copy()
        for d in (cover_dets or []):
            # Coverage boxes (never published): thin grey, for eyeballing
            # what the veto/occlusion machinery can see.
            x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (140, 140, 140), 1)
            cv2.putText(vis, "[%s]" % d["label"], (x1, min(y2 + 14, vis.shape[0] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1)
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
            "dedup_dropped=%d occlusion_dropped=%d cover_vetoed=%d "
            "search=%s detections=%s coverage=%s"
            % (s["frames"], s["requests"], s["errors"], s["skipped_interval"],
               s["skipped_spin"], avg, s["lat_max"], s["dedup_dropped"],
               s["occl_dropped"], s["cover_vetoed"], repr(self._search_term),
               s["det_counts"], s["cover_counts"]))


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
