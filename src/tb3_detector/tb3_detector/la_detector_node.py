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
import math
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
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


def containment(inner: list[float], outer: list[float]) -> float:
    """Fraction of `inner`'s area that lies inside `outer`.

    Asymmetric on purpose: it catches the PART-OF-A-WHOLE mislabel that
    IoU misses. R05 A1: at 2.4-2.9 m the model boxed only the green
    SHIRT of the centre person and labeled it "trash can" (green box,
    right apparent size — no geometric filter can reject it on its own).
    That box sits fully inside the "person wearing a green shirt"
    coverage box, but their IoU is well under 0.8 because the shirt is a
    fraction of the body, so the IoU veto never fired: the fake
    trash-can candidate promoted on the person and its cross-class mutex
    then blocked the real person observations.
    """
    ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
    ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    area_inner = (inner[2] - inner[0]) * (inner[3] - inner[1])
    return (iw * ih) / max(area_inner, 1e-9)


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
        # ── Generation options sent with every /detect (R06a step 3c) ──
        # Measured on the R05 60-frame capture (reports/R06a): greedy ==
        # sampling in latency, deterministic, reproduces 92% of the
        # sampled boxes (the rest are frame-edge person boxes the
        # localizer drops anyway); hybrid == fast on the median, but
        # hybrid's AR fallback churns on busy frames (39-48 steps, one
        # 132-step / 2.0 s outlier in the replay) where fast stays at
        # 28-29 steps. The token cap only bounds runaway generations (a
        # degenerate decode ran 6 s/frame): 32 + 40/category is ~3x a
        # real answer.
        self.declare_parameter("gen_mode", "fast")
        self.declare_parameter("gen_greedy", True)
        self.declare_parameter("gen_tokens_base", 32)
        self.declare_parameter("gen_tokens_per_category", 40)
        # Input downscale (longer side, px; 0 = native 640). Measured:
        # 448 px saves ~45 ms but halves concrete-barrier recall and drops
        # the far shirt-colour boxes — kept at native (R06a step 3b).
        self.declare_parameter("gen_max_side", 0)
        # ── Two-phase coverage (R06a step 3a, OFF) ───────────────────
        # Primary request = vocabulary + physical coverage only; the
        # listed expression terms are requested in a SECOND call on the
        # same frame only when the primary hit `two_phase_trigger`.
        # Measured on the 60-frame capture: no latency win (persons are
        # in most frames, so both calls run: 525 ms vs 409 ms) and the
        # expression-only request over-fires (blue 20 -> 31) while the
        # 5-class primary loses the barrier (8 -> 2 frames) — the model
        # answers a 9-term prompt differently from a 5-term one.
        self.declare_parameter("two_phase_coverage", False)
        self.declare_parameter("two_phase_terms", [
            "person wearing a green shirt", "person wearing a red shirt",
            "person wearing a blue shirt", "person wearing a white shirt"])
        self.declare_parameter("two_phase_trigger", "person")
        self.declare_parameter("vocabulary",
                               ["person", "bookshelf", "trash can", "traffic cone"])
        self.declare_parameter("min_interval_sec", 0.3)
        self.declare_parameter("max_angular_vel", 0.5)
        # ── Frame-skip rules (R06a step 4) ───────────────────────────
        # (a) nothing for the planar localizer to hit: no LiDAR return
        #     closer than `skip_lidar_range_m` inside the camera FOV
        #     (at least `skip_lidar_min_rays` rays, i.e. a cluster) →
        #     a box would be range-gated by memory anyway, skip.
        # (b) the view has not changed: displacement since the last
        #     request < `skip_static_dist_m` AND |yaw change| <
        #     `skip_static_yaw_deg` → skip.
        # Safety net: at least one request every `skip_fallback_sec`
        # regardless. Skip counts are in the [la_stats] line.
        self.declare_parameter("skip_lidar_enabled", True)
        self.declare_parameter("skip_lidar_range_m", 3.3)
        self.declare_parameter("skip_lidar_min_rays", 3)
        self.declare_parameter("skip_static_enabled", True)
        self.declare_parameter("skip_static_dist_m", 0.2)
        self.declare_parameter("skip_static_yaw_deg", 10.0)
        self.declare_parameter("skip_fallback_sec", 2.0)
        self.declare_parameter("camera_hfov_deg", 62.2)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cross_class_iou", 0.8)
        # Coverage veto also fires when a publishable box lies (mostly)
        # INSIDE a disagreeing coverage box — the part-of-a-whole
        # mislabel IoU cannot see (see `containment`).
        self.declare_parameter("coverage_containment", 0.8)
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
        # R05: the debug image is republished at CAMERA rate with the most
        # recent /detect overlay; boxes older than this turn grey.
        self.declare_parameter("debug_stale_after_sec", 1.5)
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
        self._gen_mode = str(g("gen_mode"))
        self._gen_greedy = bool(g("gen_greedy"))
        self._gen_tokens_base = int(g("gen_tokens_base"))
        self._gen_tokens_per_cat = int(g("gen_tokens_per_category"))
        self._gen_max_side = int(g("gen_max_side"))
        self._two_phase = bool(g("two_phase_coverage"))
        self._two_phase_terms = [str(t).lower() for t in g("two_phase_terms")]
        self._two_phase_trigger = str(g("two_phase_trigger")).lower()
        self._min_interval = float(g("min_interval_sec"))
        self._max_ang_vel = float(g("max_angular_vel"))
        self._skip_lidar = bool(g("skip_lidar_enabled"))
        self._skip_lidar_range = float(g("skip_lidar_range_m"))
        self._skip_lidar_min_rays = int(g("skip_lidar_min_rays"))
        self._skip_static = bool(g("skip_static_enabled"))
        self._skip_static_dist = float(g("skip_static_dist_m"))
        self._skip_static_yaw = math.radians(float(g("skip_static_yaw_deg")))
        self._skip_fallback = float(g("skip_fallback_sec"))
        self._half_hfov = math.radians(float(g("camera_hfov_deg"))) / 2.0
        self._cross_iou = float(g("cross_class_iou"))
        self._cover_containment = float(g("coverage_containment"))
        self._occlusion_filter = bool(g("occlusion_filter"))
        self._pub_debug = bool(g("publish_debug_image"))
        self._debug_stale_after = float(g("debug_stale_after_sec"))
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
        self._pose: tuple[float, float, float] | None = None   # x, y, yaw (odom)
        self._last_request_pose: tuple[float, float, float] | None = None
        self._latest_scan: LaserScan | None = None
        self._server_seen = False
        self._options_logged = False
        # R05 camera-rate debug: /detect runs in a worker thread so the
        # image callback never blocks on HTTP; the callback republishes
        # the debug image EVERY frame with the latest finished overlay.
        # `_inflight` serializes requests — the detection cadence stays
        # one request per max(min_interval, round-trip), same as before.
        self._inflight = threading.Event()
        self._overlay: dict | None = None   # swapped atomically by worker

        # Rolling stats for the [la_stats] line (report metrics: per-frame
        # /detect latency and cross-class dedup count).
        self._st = dict(frames=0, skipped_interval=0, skipped_spin=0,
                        skipped_lidar=0, skipped_static=0, forced_fallback=0,
                        requests=0, second_phase=0, errors=0, dedup_dropped=0,
                        srv_sum=0.0, srv_n=0,
                        occl_dropped=0, cover_vetoed=0, lat_sum=0.0,
                        lat_max=0.0, det_counts={}, cover_counts={})
        self._last_stats_wall = time.monotonic()
        self._t_start = time.monotonic()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=queue)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE, depth=queue)

        self.create_subscription(Image, str(g("image_topic")),
                                 self._image_cb, sensor_qos)
        self.create_subscription(Odometry, str(g("odom_topic")),
                                 self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, str(g("scan_topic")),
                                 self._scan_cb, sensor_qos)
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
            "min_interval=%.2fs max_ang_vel=%.2frad/s cross_iou=%.2f "
            "gen=%s/%s max_side=%d two_phase=%s"
            % (self._server_url, self._vocabulary, self._min_interval,
               self._max_ang_vel, self._cross_iou, self._gen_mode,
               "greedy" if self._gen_greedy else "sample",
               self._gen_max_side, self._two_phase))

    # ── Search-link vocabulary injection ────────────────────────────────

    def _rebuild_vocabulary(self) -> None:
        """Recompute the effective vocabulary and dedup priorities.

        The search term (if any) goes FIRST: index 0 is the best (lowest)
        priority in dedup_cross_class, so a search-term box always wins a
        cross-class overlap against base-vocabulary boxes (Phase 2c
        constraint a). A term that equals a base word is not duplicated.
        """
        base = self._base_vocabulary
        if self._search_term:
            # A base word that is a token-subset of the search term is
            # SUSPENDED for the duration of the search. Measured (R05
            # green-shirt probe): with "person" in the vocabulary the
            # model answers almost every frame with the generic label —
            # 503 "person" observations against 18 for the injected
            # "person wearing a green shirt", nowhere near a promotion.
            # R04's search-first profile succeeded on 11 observations
            # precisely because the generic word was absent and the
            # model had to choose between the expression and the color
            # coverage classes. Suspending (not deleting) keeps mapping
            # for every other class running, and the word returns the
            # moment the search clears.
            base = [w for w in base if not labels_agree(w, self._search_term)]
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
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ── Skip rules ──────────────────────────────────────────────────────

    def _fov_clear(self) -> bool:
        """True when fewer than `skip_lidar_min_rays` LiDAR returns inside
        the camera FOV are closer than `skip_lidar_range_m` (scan angles
        run [0, 2π) with 0 = forward, as in the localizer)."""
        sc = self._latest_scan
        if sc is None or sc.angle_increment == 0.0:
            return False
        near = 0
        for i, r in enumerate(sc.ranges):
            a = sc.angle_min + i * sc.angle_increment
            a = (a + math.pi) % (2.0 * math.pi) - math.pi   # → (-π, π]
            if abs(a) > self._half_hfov:
                continue
            if math.isfinite(r) and sc.range_min <= r <= self._skip_lidar_range:
                near += 1
                if near >= self._skip_lidar_min_rays:
                    return False
        return True

    def _view_unchanged(self) -> bool:
        if self._pose is None or self._last_request_pose is None:
            return False
        x, y, yaw = self._pose
        lx, ly, lyaw = self._last_request_pose
        dyaw = (yaw - lyaw + math.pi) % (2.0 * math.pi) - math.pi
        return (math.hypot(x - lx, y - ly) < self._skip_static_dist
                and abs(dyaw) < self._skip_static_yaw)

    def _image_cb(self, msg: Image) -> None:
        self._st["frames"] += 1
        now = self.get_clock().now().nanoseconds * 1e-9

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning("cv_bridge conversion failed: %s" % e)
            return

        # Camera-rate debug: every frame goes out with the most recent
        # finished overlay, regardless of whether a request fires below.
        if self._dbg_pub is not None:
            self._publish_debug(bgr, msg.header)

        if self._inflight.is_set():
            self._st["skipped_interval"] += 1
            return
        if (self._last_request_t is not None
                and now - self._last_request_t < self._min_interval):
            self._st["skipped_interval"] += 1
            return
        if abs(self._angular_vel) > self._max_ang_vel:
            self._st["skipped_spin"] += 1
            return
        # Skip rules (R06a step 4), overridden by the periodic fallback.
        forced = (self._last_request_t is not None
                  and now - self._last_request_t >= self._skip_fallback)
        if not forced:
            if self._skip_lidar and self._fov_clear():
                self._st["skipped_lidar"] += 1
                return
            if self._skip_static and self._view_unchanged():
                self._st["skipped_static"] += 1
                return
        elif self._skip_lidar or self._skip_static:
            self._st["forced_fallback"] += 1

        # Mark the slot BEFORE dispatching: queued frames that drain while
        # the worker runs must not fire immediately again.
        self._last_request_t = now
        self._last_request_pose = self._pose
        self._inflight.set()
        threading.Thread(target=self._detect_worker,
                         args=(bgr.copy(), msg.header), daemon=True).start()

    def _detect_worker(self, bgr, header) -> None:
        try:
            self._run_detect(bgr, header)
        except Exception as e:
            self.get_logger().error("detect worker failed: %s" % e)
        finally:
            self._inflight.clear()

    def _run_detect(self, bgr, header) -> None:
        t0 = time.monotonic()
        categories = self._vocabulary + self._coverage
        deferred: list[str] = []
        if self._two_phase:
            deferred = [c for c in categories if c in self._two_phase_terms
                        and c not in self._priority]
            categories = [c for c in categories if c not in deferred]
        result = self._post_detect(bgr, categories)
        if result is None:
            return
        if deferred and any(self._two_phase_trigger in d.get("label", "")
                            for d in result.get("detections", [])):
            second = self._post_detect(bgr, deferred)
            self._st["second_phase"] += 1
            if second is not None:
                result["detections"] = (result.get("detections", [])
                                        + second.get("detections", []))
        rtt_ms = (time.monotonic() - t0) * 1000.0
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
                clash = any(
                    (iou_xyxy(c["bbox_xyxy"], d["bbox_xyxy"]) >= self._cross_iou
                     or containment(d["bbox_xyxy"], c["bbox_xyxy"])
                     >= self._cover_containment)
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

        self._det_pub.publish(self._to_msg(dets, header))
        # Atomic swap read by the image callback; wall time (monotonic)
        # drives the staleness grey-out.
        self._overlay = dict(dets=dets, cover_dets=cover_dets,
                             rtt_ms=rtt_ms, wall_t=time.monotonic())
        self._maybe_log_stats()

    # ── HTTP ────────────────────────────────────────────────────────────

    def _post_detect(self, bgr, categories: list[str]) -> dict | None:
        ok, buf = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            self.get_logger().warning("JPEG encode failed, skipping frame")
            return None
        payload = json.dumps({
            "image": base64.b64encode(buf.tobytes()).decode("ascii"),
            "categories": categories,
            "options": {
                "generation_mode": self._gen_mode,
                "do_sample": not self._gen_greedy,
                "max_new_tokens": self._gen_tokens_base
                + self._gen_tokens_per_cat * len(categories),
                "max_side": self._gen_max_side,
            },
        }).encode()
        req = urllib.request.Request(
            self._server_url + "/detect", data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                out = json.load(resp)
            timing = out.get("timing") or {}
            if not self._options_logged:
                self._options_logged = True
                self.get_logger().info(
                    "grounding server responding (backend=%s, options in "
                    "force=%s)" % (out.get("backend"), timing.get("options")))
            self._st["srv_sum"] += float(timing.get("total_ms", out.get("latency_ms", 0.0)))
            self._st["srv_n"] += 1
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

    def _publish_debug(self, bgr, header) -> None:
        """Camera-rate debug frame: current image + the LAST finished
        /detect overlay. Boxes older than `debug_stale_after_sec` turn
        grey — during the ~0.5 s round-trip the boxes lag the scene, and
        past 1.5 s (server down, spin gate) they are visibly outdated."""
        vis = bgr.copy()
        ov = self._overlay
        if ov is None:
            cv2.putText(vis, "LA no detection yet", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            age = time.monotonic() - ov["wall_t"]
            stale = age > self._debug_stale_after
            det_color = (160, 160, 160) if stale else (255, 120, 0)
            for d in ov["cover_dets"]:
                # Coverage boxes (never published): thin grey, for
                # eyeballing what the veto/occlusion machinery can see.
                x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
                cv2.rectangle(vis, (x1, y1), (x2, y2), (140, 140, 140), 1)
                cv2.putText(vis, "[%s]" % d["label"],
                            (x1, min(y2 + 14, vis.shape[0] - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1)
            for d in ov["dets"]:
                x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
                cv2.rectangle(vis, (x1, y1), (x2, y2), det_color, 2)
                cv2.putText(vis, d["label"], (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, det_color, 2)
            cv2.putText(vis, "LA /detect %.0f ms | boxes %.1fs ago%s"
                        % (ov["rtt_ms"], age, " (STALE)" if stale else ""),
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1)
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
        srv = s["srv_sum"] / s["srv_n"] if s["srv_n"] else 0.0
        self.get_logger().info(
            "[la_stats] frames=%d requests=%d(+%d second-phase) errors=%d "
            "skipped(interval=%d, spin=%d, lidar_clear=%d, static=%d; "
            "fallback_forced=%d) req_rate=%.2f/s "
            "latency(avg=%.0fms, max=%.0fms, server avg=%.0fms) "
            "dedup_dropped=%d occlusion_dropped=%d cover_vetoed=%d "
            "search=%s detections=%s coverage=%s"
            % (s["frames"], s["requests"], s["second_phase"], s["errors"],
               s["skipped_interval"], s["skipped_spin"], s["skipped_lidar"],
               s["skipped_static"], s["forced_fallback"],
               # requests per wall second since the node started: the
               # skip rules show up as a lower rate than 1/round-trip
               s["requests"] / max(1e-6, now - self._t_start),
               avg, s["lat_max"], srv, s["dedup_dropped"],
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
