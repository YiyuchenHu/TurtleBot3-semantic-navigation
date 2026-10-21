#!/usr/bin/env python3
"""
semantic_map_memory_node.py — Persistent semantic landmark memory with
occupancy-grid validation, wall-island rejection, and obstacle-island refinement.

Pipeline per observation:
  1. TF transform base_link -> map
  2. Reject if outside occupancy grid bounds
  3. Search local window for occupied cells; BFS connected components
  4. Reject wall-like islands (touches grid border or centroid near boundary)
  5. Snap observation to nearest valid island centroid
  6. Match refined point against existing landmarks/candidates (same class + distance)
  7. Candidates promoted to persistent landmarks after min_observations
  8. Publish all persistent landmarks as MarkerArray

Every rejection in that pipeline (range, TF, grid bounds, no island,
geometry, mutex) and every success (landmark merge, candidate merge, new
candidate, promotion) is counted per class, together with the key numbers
of the most recent event, and printed as one `[gate_summary]` block every
`gate_summary_interval_sec` seconds when something changed (plus a final
one at shutdown). Nothing is decided from those counters; they exist so a
log can say which gate held a target back and by how much.
"""

from __future__ import annotations

import math
import time as _time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


@dataclass
class Candidate:
    semantic_class: str
    x: float
    y: float
    obs_count: int = 1
    last_seen: float = 0.0
    # Observation only: short-term-memory id -> observations contributed.
    feeders: dict = field(default_factory=dict)

    def feeder_summary(self, top=3):
        """'k ids, top X% : id:n,id:n' for the log."""
        if not self.feeders:
            return "no ids"
        total = sum(self.feeders.values())
        ranked = sorted(self.feeders.items(), key=lambda kv: -kv[1])
        return "%d ids, top %.0f%%: %s" % (
            len(ranked), 100.0 * ranked[0][1] / max(1, total),
            ",".join("%s:%d" % kv for kv in ranked[:top]))


@dataclass
class Landmark:
    landmark_id: str
    semantic_class: str
    x: float
    y: float
    last_seen: float = 0.0
    # Evidence per detector label observed inside this landmark's merge
    # radius, INCLUDING labels that disagree with `semantic_class`. Before
    # this existed the cross-class mutex threw disagreeing observations away,
    # so a landmark that was promoted under the wrong label could never be
    # corrected: the correct observations were the ones being discarded
    # (measured 2026-09-05: 86 `chair` observations rejected by a
    # `trash_can_1` landmark sitting on the chair). Keeping the counts turns
    # that same disagreement into the evidence for a relabel.
    class_counts: dict = field(default_factory=dict)

    @property
    def observation_count(self):
        """Observations supporting the CURRENT label.

        This is what the position EMA, the marker text, the mutex strength
        and the published hypothesis score all mean by "n" - a landmark with
        12 trash_can and 86 chair observations still has n=12 while it is
        labelled trash_can.
        """
        return self.class_counts.get(self.semantic_class, 0)

    def counts_summary(self):
        """'chair:86,trash_can:12' - strongest label first."""
        if not self.class_counts:
            return "-"
        return ",".join(
            "%s:%d" % kv
            for kv in sorted(self.class_counts.items(), key=lambda kv: (-kv[1], kv[0])))


# Keyed by DETECTOR LABEL, because that is what `Landmark.semantic_class`
# carries and what the landmark ids are built from (`trash_can_0`). The three
# keys are the enabled detector_label values in
# tb3_bringup/config/semantic_targets.yaml; anything else falls back to grey.
CLASS_COLORS = {
    "person":    ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9),   # green
    "trash_can": ColorRGBA(r=0.2, g=0.45, b=0.95, a=0.9),  # blue
    "chair":     ColorRGBA(r=0.95, g=0.55, b=0.1, a=0.9),  # orange
}
DEFAULT_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9)


def world_to_grid(wx, wy, ox, oy, res):
    return int((wx - ox) / res), int((wy - oy) / res)


def grid_to_world(gx, gy, ox, oy, res):
    return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res


def find_nearest_valid_island(
    grid, width, height, cx, cy, search_radius,
    origin_x, origin_y, resolution,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
    diag=None,
):
    """BFS flood-fill for obstacle islands, rejecting wall-like islands.

    `diag`, if a dict, receives counts of what the window held: occupied
    cells, islands found, and how many were dropped as too small, touching
    the border, or inside the wall margin. Pure observation; the return
    value does not depend on it.
    """
    x_lo = max(0, cx - search_radius)
    x_hi = min(width, cx + search_radius + 1)
    y_lo = max(0, cy - search_radius)
    y_hi = min(height, cy + search_radius + 1)

    visited = set()
    islands = []
    n_occupied = 0
    n_small = 0
    n_border = 0
    n_wall = 0

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            n_occupied += len(island)
            if len(island) < min_island_pixels:
                n_small += 1
                continue
            if touches_border:
                n_border += 1
                continue
            n = len(island)
            avg_gx = sum(p[0] for p in island) / n
            avg_gy = sum(p[1] for p in island) / n
            if (avg_gx < wall_margin_cells or avg_gx >= width - wall_margin_cells or
                    avg_gy < wall_margin_cells or avg_gy >= height - wall_margin_cells):
                n_wall += 1
                continue
            islands.append((avg_gx, avg_gy))

    if diag is not None:
        diag["occupied"] = n_occupied
        diag["islands"] = n_small + n_border + n_wall + len(islands)
        diag["small"] = n_small
        diag["border"] = n_border
        diag["wall"] = n_wall
        diag["valid"] = len(islands)

    if not islands:
        return None
    best = min(islands, key=lambda t: math.hypot(t[0] - cx, t[1] - cy))
    return grid_to_world(int(best[0]), int(best[1]), origin_x, origin_y, resolution)



def find_bench_cluster_centroid(
    grid, width, height, cx, cy, search_radius,
    origin_x, origin_y, resolution,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
    cluster_radius_cells=8,
    logger=None,
):
    """For bench-like objects: cluster nearby valid islands and return
    the size-weighted centroid of the cluster nearest to the observation."""
    x_lo = max(0, cx - search_radius)
    x_hi = min(width, cx + search_radius + 1)
    y_lo = max(0, cy - search_radius)
    y_hi = min(height, cy + search_radius + 1)

    visited = set()
    valid_islands = []  # (avg_gx, avg_gy, pixel_count)

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            if len(island) < min_island_pixels or touches_border:
                continue
            n = len(island)
            agx = sum(p[0] for p in island) / n
            agy = sum(p[1] for p in island) / n
            if (agx < wall_margin_cells or agx >= width - wall_margin_cells or
                    agy < wall_margin_cells or agy >= height - wall_margin_cells):
                continue
            valid_islands.append((agx, agy, n))

    if not valid_islands:
        return None

    # Find islands close to the observation point
    nearby = [(gx, gy, n) for gx, gy, n in valid_islands
              if math.hypot(gx - cx, gy - cy) <= cluster_radius_cells]

    if not nearby:
        # Fallback: nearest single island
        best = min(valid_islands, key=lambda t: math.hypot(t[0] - cx, t[1] - cy))
        pt = grid_to_world(int(best[0]), int(best[1]), origin_x, origin_y, resolution)
        if logger:
            logger.info("[bench] fallback nearest island at (%.2f, %.2f)" % pt)
        return pt

    if len(nearby) == 1:
        pt = grid_to_world(int(nearby[0][0]), int(nearby[0][1]),
                           origin_x, origin_y, resolution)
        if logger:
            logger.info("[bench] single island at (%.2f, %.2f)  %d px" % (pt[0], pt[1], nearby[0][2]))
        return pt

    # Multi-island cluster: size-weighted centroid
    total_px = sum(t[2] for t in nearby)
    wgx = sum(t[0] * t[2] for t in nearby) / total_px
    wgy = sum(t[1] * t[2] for t in nearby) / total_px
    pt = grid_to_world(int(wgx), int(wgy), origin_x, origin_y, resolution)
    if logger:
        logger.info("[bench] cluster %d islands  %d total_px  centroid (%.2f, %.2f)"
                    % (len(nearby), total_px, pt[0], pt[1]))
    return pt



def classify_local_geometry(
    grid, width, height, cx, cy, radius_cells,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
):
    """Classify the local obstacle geometry around (cx, cy) in grid coords.

    Returns (n_islands, total_pixels, max_island_pixels, aspect_ratio).
    - n_islands: number of valid (non-wall) islands within radius
    - total_pixels: sum of all valid island pixels
    - max_island_pixels: size of the largest island
    - aspect_ratio: bounding-box width/height of the largest island (>1 = wide)
    """
    x_lo = max(0, cx - radius_cells)
    x_hi = min(width, cx + radius_cells + 1)
    y_lo = max(0, cy - radius_cells)
    y_hi = min(height, cy + radius_cells + 1)

    visited = set()
    islands = []

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island_cells = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island_cells.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            if len(island_cells) < min_island_pixels or touches_border:
                continue
            n = len(island_cells)
            agx = sum(p[0] for p in island_cells) / n
            agy = sum(p[1] for p in island_cells) / n
            if (agx < wall_margin_cells or agx >= width - wall_margin_cells or
                    agy < wall_margin_cells or agy >= height - wall_margin_cells):
                continue
            xs = [p[0] for p in island_cells]
            ys = [p[1] for p in island_cells]
            bbox_w = max(xs) - min(xs) + 1
            bbox_h = max(ys) - min(ys) + 1
            aspect = max(bbox_w, bbox_h) / max(min(bbox_w, bbox_h), 1)
            islands.append((n, aspect))

    if not islands:
        return 0, 0, 0, 1.0

    n_islands = len(islands)
    total_px = sum(i[0] for i in islands)
    max_px = max(i[0] for i in islands)
    max_aspect = max(i[1] for i in islands)
    return n_islands, total_px, max_px, max_aspect


def check_geometry_consistency(
    label, n_islands, total_pixels, max_island_px, aspect_ratio,
    person_max_islands=2, person_max_total_px=20,
    bench_min_total_px=8, bench_min_islands_or_wide=True,
):
    """Return (compatible, reason) for the detector label vs local geometry.

    compatible=True  -> observation is allowed
    compatible=False -> observation should be rejected
    """
    if label == "person":
        if n_islands > person_max_islands and total_pixels > person_max_total_px:
            return False, (
                "person obs but %d islands, %d px (bench-like geometry)"
                % (n_islands, total_pixels))
        if max_island_px > person_max_total_px and aspect_ratio > 2.5:
            return False, (
                "person obs but largest island %d px, aspect %.1f (wide/bench-like)"
                % (max_island_px, aspect_ratio))
        return True, "person-compatible (%d islands, %d px)" % (n_islands, total_pixels)

    if label == "bench":
        if n_islands == 1 and total_pixels < bench_min_total_px and aspect_ratio < 1.5:
            return False, (
                "bench obs but only 1 small compact island (%d px, aspect %.1f, person-like)"
                % (total_pixels, aspect_ratio))
        return True, "bench-compatible (%d islands, %d px, aspect %.1f)" % (
            n_islands, total_pixels, aspect_ratio)

    return True, "no geometry check for class '%s'" % label


def should_relabel(class_counts, current_class, relabel_ratio,
                   class_min_obs, default_min_obs=3):
    """Pure rule: which label should replace `current_class`, or None.

    A challenger label takes over only when BOTH hold:

      * it has at least `relabel_ratio` times as many observations as the
        current label - the hysteresis that stops two labels flip-flopping
        while their counts are close, and
      * it has reached its own `min_observations` promotion threshold, so a
        handful of stray detections cannot rename an established landmark.

    Equal counts never relabel (ratio >= 1.0). Because the losing label keeps
    its count, flipping back needs `relabel_ratio` times the NEW label's
    count, which is what makes the correction stable rather than oscillating.

    Ties between two qualifying challengers go to the larger count, then to
    the alphabetically first label, so the outcome never depends on dict
    ordering.
    """
    current = class_counts.get(current_class, 0)
    best_label = None
    best_n = -1
    for label, n in sorted(class_counts.items()):
        if label == current_class:
            continue
        if n < class_min_obs.get(label, default_min_obs):
            continue
        if n < relabel_ratio * current:
            continue
        if n > best_n:
            best_label, best_n = label, n
    return best_label


class SemanticMapMemoryNode(Node):

    # Rows of the gate summary, in pipeline order. The first six are the
    # rejection gates of _obs_cb (the four that used to be silent, plus the
    # two that already log per event); the last four are the success paths.
    GATE_ROWS = ("range", "tf", "out_of_grid", "no_island", "geometry", "mutex")
    PASS_ROWS = ("merge_landmark", "cross_evidence", "relabel",
                 "merge_candidate", "new_candidate", "promote")
    STAT_ROWS = GATE_ROWS + PASS_ROWS

    def __init__(self):
        super().__init__("semantic_map_memory_node")

        self.declare_parameter("input_topic", "/semantic_memory_node/objects")
        self.declare_parameter("output_topic", "/semantic_memory_markers")
        self.declare_parameter("landmark_objects_topic", "/semantic_map_memory_node/landmark_objects")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("merge_distance", 1.5)
        self.declare_parameter("candidate_merge_distance", 1.5)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("search_radius_cells", 20)
        self.declare_parameter("sphere_radius", 0.18)
        self.declare_parameter("text_offset_z", 0.45)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("candidate_timeout", 45.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_island_pixels", 2)
        self.declare_parameter("wall_boundary_margin_m", 0.20)
        self.declare_parameter("bench_cluster_radius_cells", 10)
        self.declare_parameter("max_observation_range_m", 2.5)
        self.declare_parameter("person_max_range_m", 2.5)
        self.declare_parameter("chair_max_range_m", 3.2)
        self.declare_parameter("trash_can_max_range_m", 3.2)
        self.declare_parameter("person_min_observations", 8)
        self.declare_parameter("chair_min_observations", 12)
        self.declare_parameter("trash_can_min_observations", 12)
        self.declare_parameter("geometry_check_enabled", True)
        self.declare_parameter("geometry_check_radius_cells", 12)
        self.declare_parameter("person_max_islands", 2)
        self.declare_parameter("person_max_total_px", 20)
        self.declare_parameter("bench_min_total_px", 8)
        self.declare_parameter("relabel_enabled", True)
        self.declare_parameter("relabel_ratio", 1.5)
        self.declare_parameter("relabel_evidence_distance_m", 0.6)
        self.declare_parameter("cross_class_mutex_enabled", True)
        self.declare_parameter("cross_class_mutex_distance_m", 0.6)
        self.declare_parameter("mutex_min_observation_count", 3)
        # Observability only: how often the per-gate counters are printed.
        # A block is printed only when a counter changed since the last one,
        # so an idle node stays quiet. 0 disables the periodic block (the
        # final one at shutdown is still printed).
        self.declare_parameter("gate_summary_interval_sec", 10.0)

        in_topic        = self.get_parameter("input_topic").value
        out_topic       = self.get_parameter("output_topic").value
        map_topic       = self.get_parameter("map_topic").value
        self._frame     = self.get_parameter("target_frame").value
        self._tf_tout   = self.get_parameter("tf_timeout").value
        self._merge_d   = self.get_parameter("merge_distance").value
        self._cand_d    = self.get_parameter("candidate_merge_distance").value
        self._min_obs   = self.get_parameter("min_observations").value
        self._search_r  = self.get_parameter("search_radius_cells").value
        self._sphere_r  = self.get_parameter("sphere_radius").value
        self._text_z    = self.get_parameter("text_offset_z").value
        pub_rate        = self.get_parameter("publish_rate").value
        self._cand_tout = self.get_parameter("candidate_timeout").value
        self._occ_thresh = self.get_parameter("occupied_threshold").value
        self._min_island = self.get_parameter("min_island_pixels").value
        self._wall_margin_m = self.get_parameter("wall_boundary_margin_m").value
        self._bench_cluster_r = self.get_parameter("bench_cluster_radius_cells").value
        self._max_obs_range = self.get_parameter("max_observation_range_m").value

        self._class_max_range = {
            "person": self.get_parameter("person_max_range_m").value,
            "chair": self.get_parameter("chair_max_range_m").value,
            "trash_can": self.get_parameter("trash_can_max_range_m").value,
        }
        self._class_min_obs = {
            "person": self.get_parameter("person_min_observations").value,
            "chair": self.get_parameter("chair_min_observations").value,
            "trash_can": self.get_parameter("trash_can_min_observations").value,
        }

        self._geom_enabled = self.get_parameter("geometry_check_enabled").value
        self._geom_radius = self.get_parameter("geometry_check_radius_cells").value
        self._person_max_islands = self.get_parameter("person_max_islands").value
        self._person_max_px = self.get_parameter("person_max_total_px").value
        self._bench_min_px = self.get_parameter("bench_min_total_px").value

        self._relabel_enabled = self.get_parameter("relabel_enabled").value
        self._relabel_ratio = float(self.get_parameter("relabel_ratio").value)
        self._evidence_d = float(
            self.get_parameter("relabel_evidence_distance_m").value)
        self._mutex_enabled = self.get_parameter("cross_class_mutex_enabled").value
        self._mutex_dist = self.get_parameter("cross_class_mutex_distance_m").value
        self._mutex_min_obs = self.get_parameter("mutex_min_observation_count").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos_rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Detection3DArray, in_topic, self._obs_cb, qos_rel)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, qos_map)
        self._pub = self.create_publisher(MarkerArray, out_topic, qos_rel)

        lm_obj_topic = self.get_parameter("landmark_objects_topic").value
        self._lm_obj_pub = self.create_publisher(Detection3DArray, lm_obj_topic, qos_rel)

        self._map_data = None
        self._map_width = 0
        self._map_height = 0
        self._map_res = 0.05
        self._map_ox = 0.0
        self._map_oy = 0.0

        self._candidates = []
        self._landmarks = {}
        self._next_seq = {}

        # ── Gate statistics (observation only, never consulted) ─────────
        # One counter per (gate, class) for every `continue` in _obs_cb and
        # every success path, plus the key numbers of the last event per
        # (gate, class) so the summary can say by how much a gate failed.
        self._stat_msgs = 0            # Detection3DArray messages received
        self._stat_msgs_no_map = 0     # ... dropped because /map not yet seen
        self._stat_in = defaultdict(int)          # observations per class
        self._stat = {name: defaultdict(int) for name in self.STAT_ROWS}
        self._stat_last = {name: {} for name in self.STAT_ROWS}
        self._stat_expired = defaultdict(int)     # candidates timed out
        self._stat_expired_max_n = defaultdict(int)
        self._stat_expired_last = {}
        self._stat_t0 = _time.time()
        self._stat_printed = None      # snapshot at the last printed block

        self.create_timer(1.0 / max(pub_rate, 0.1), self._publish_markers)
        self.create_timer(5.0, self._cleanup_candidates)
        summary_dt = float(self.get_parameter("gate_summary_interval_sec").value)
        if summary_dt > 0.0:
            self.create_timer(summary_dt, self._log_gate_summary)

        self.get_logger().info(
            "SemanticMapMemoryNode ready  merge=%.2fm  cand=%.2fm  min_obs=%d  "
            "search_r=%d  wall_margin=%.2fm  min_island=%d  relabel=%s(x%.2f)"
            % (self._merge_d, self._cand_d, self._min_obs,
               self._search_r, self._wall_margin_m, self._min_island,
               "on" if self._relabel_enabled else "off", self._relabel_ratio))

    def _map_cb(self, msg):
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_res = msg.info.resolution
        self._map_ox = msg.info.origin.position.x
        self._map_oy = msg.info.origin.position.y
        self._map_data = np.array(msg.data, dtype=np.int8)

    def _obs_cb(self, msg):
        self._stat_msgs += 1
        if self._map_data is None:
            self._stat_msgs_no_map += 1
            return

        wall_margin_cells = max(1, int(self._wall_margin_m / self._map_res))

        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            src = det.header.frame_id or msg.header.frame_id or "base_link"
            obs_id = det.id or "-"
            self._stat_in[label] += 1

            raw_x = det.bbox.center.position.x
            raw_y = det.bbox.center.position.y
            obs_range = math.hypot(raw_x, raw_y)

            class_max = self._class_max_range.get(label, self._max_obs_range)
            if obs_range > class_max:
                self._note("range", label,
                           "r=%.2f > max %.2f (+%.2f) raw=(%.2f,%.2f) id=%s"
                           % (obs_range, class_max, obs_range - class_max,
                              raw_x, raw_y, obs_id))
                continue

            pt_in = PointStamped()
            pt_in.header.stamp = rclpy.time.Time(seconds=0).to_msg()
            pt_in.header.frame_id = src
            pt_in.point.x = raw_x
            pt_in.point.y = raw_y

            try:
                pt_out = self._tf_buffer.transform(
                    pt_in, self._frame,
                    timeout=Duration(seconds=self._tf_tout))
            except Exception as exc:  # noqa: BLE001 - same catch as before
                self._note("tf", label,
                           "%s->%s %s: %s id=%s"
                           % (src, self._frame, type(exc).__name__,
                              str(exc).replace("\n", " ")[:90], obs_id))
                continue

            mx, my = pt_out.point.x, pt_out.point.y
            gx, gy = world_to_grid(mx, my, self._map_ox, self._map_oy, self._map_res)

            if not (0 <= gx < self._map_width and 0 <= gy < self._map_height):
                self._note("out_of_grid", label,
                           "map=(%.2f,%.2f) cell=(%d,%d) grid=%dx%d id=%s"
                           % (mx, my, gx, gy, self._map_width,
                              self._map_height, obs_id))
                continue

            if label == "bench":
                refined = find_bench_cluster_centroid(
                    self._map_data, self._map_width, self._map_height,
                    gx, gy, self._search_r,
                    self._map_ox, self._map_oy, self._map_res,
                    occupied_thresh=self._occ_thresh,
                    min_island_pixels=self._min_island,
                    wall_margin_cells=wall_margin_cells,
                    cluster_radius_cells=self._bench_cluster_r,
                    logger=self.get_logger())
            else:
                island_diag = {}
                refined = find_nearest_valid_island(
                    self._map_data, self._map_width, self._map_height,
                    gx, gy, self._search_r,
                    self._map_ox, self._map_oy, self._map_res,
                    occupied_thresh=self._occ_thresh,
                    min_island_pixels=self._min_island,
                    wall_margin_cells=wall_margin_cells,
                    diag=island_diag)

            if refined is None:
                if label == "bench":
                    why = "bench cluster search found no island"
                else:
                    why = ("window r=%d cells: occ=%d islands=%d (small %d, "
                           "border %d, wall_margin %d)"
                           % (self._search_r, island_diag.get("occupied", 0),
                              island_diag.get("islands", 0),
                              island_diag.get("small", 0),
                              island_diag.get("border", 0),
                              island_diag.get("wall", 0)))
                self._note("no_island", label,
                           "map=(%.2f,%.2f) r=%.2f %s id=%s"
                           % (mx, my, obs_range, why, obs_id))
                continue

            rx, ry = refined

            if self._geom_enabled and label in ("person", "bench"):
                n_isl, tot_px, max_px, aspect = classify_local_geometry(
                    self._map_data, self._map_width, self._map_height,
                    gx, gy, self._geom_radius,
                    occupied_thresh=self._occ_thresh,
                    min_island_pixels=self._min_island,
                    wall_margin_cells=wall_margin_cells)
                compat, reason = check_geometry_consistency(
                    label, n_isl, tot_px, max_px, aspect,
                    person_max_islands=self._person_max_islands,
                    person_max_total_px=self._person_max_px,
                    bench_min_total_px=self._bench_min_px)
                if not compat:
                    self.get_logger().info("[reject] %s" % reason)
                    self._note("geometry", label, reason)
                    continue
                self.get_logger().debug("[geometry] %s" % reason)

            blocked, mutex_reason = self._check_cross_class_mutex(label, rx, ry)
            if blocked:
                self.get_logger().info("[mutex] %s" % mutex_reason)
                self._note("mutex", label, "%s id=%s" % (mutex_reason, obs_id))
                continue

            now = _time.time()

            matched_lm = self._find_landmark(label, rx, ry)
            if matched_lm is not None:
                d_lm = math.hypot(matched_lm.x - rx, matched_lm.y - ry)
                self._update_landmark(matched_lm, rx, ry, now)
                self._note("merge_landmark", label,
                           "%s n=%d d=%.2f id=%s"
                           % (matched_lm.landmark_id,
                              matched_lm.observation_count, d_lm, obs_id))
                continue

            other_lm = self._find_landmark_other_class(label, rx, ry)
            if other_lm is not None:
                self._record_cross_evidence(other_lm, label, rx, ry, now)
                continue

            matched_cand = self._find_candidate(label, rx, ry)
            if matched_cand is not None:
                d_c = math.hypot(matched_cand.x - rx, matched_cand.y - ry)
                self._update_candidate(matched_cand, rx, ry, now, obs_id)
                class_min_obs = self._class_min_obs.get(label, self._min_obs)
                self._note("merge_candidate", label,
                           "n=%d/%d d=%.2f pos=(%.2f,%.2f) id=%s"
                           % (matched_cand.obs_count, class_min_obs, d_c,
                              matched_cand.x, matched_cand.y, obs_id))
                if matched_cand.obs_count >= class_min_obs:
                    self._note("promote", label,
                               "n=%d pos=(%.2f,%.2f)"
                               % (matched_cand.obs_count,
                                  matched_cand.x, matched_cand.y))
                    self._promote(matched_cand)
                continue

            self._note("new_candidate", label,
                       "pos=(%.2f,%.2f) r=%.2f id=%s" % (rx, ry, obs_range, obs_id))
            self._candidates.append(Candidate(
                semantic_class=label, x=rx, y=ry, obs_count=1, last_seen=now,
                feeders={obs_id: 1}))

    def _check_cross_class_mutex(self, label, x, y):
        """Check if a strong different-class CANDIDATE sits at this location.

        Returns (blocked, reason); blocked=True means the observation is
        rejected because a stronger different-class candidate exists nearby.

        Landmarks are deliberately not consulted any more. A disagreeing
        observation that lands on an existing landmark is now recorded as
        evidence against that landmark's label (see `_record_cross_evidence`)
        instead of being discarded, which is what makes a mislabelled
        landmark recoverable. The candidate half of the rule is unchanged:
        it is the gate that stopped label swaps from ever being promoted in
        the first place, and it is still doing that job.
        """
        if not self._mutex_enabled:
            return False, ""
        # Labels this gate applies to: the raw detector labels of the three
        # targets. Keep every target that can be confused with another one
        # here: measured 2026-08-27 with the COCO weights, yolo26n swapped
        # "chair" and the trash can whenever either sat at the edge of the frame
        # partially clipped, which planted a chair landmark on the trash can and
        # vice versa. The fine-tuned weights have not shown the swap, but the
        # mutex is cheap insurance: the entry with more observations wins, and
        # the genuine one always has far more.
        if label not in ("person", "chair", "trash_can"):
            return False, ""

        best_blocker = None
        best_obs = 0

        for c in self._candidates:
            if c.semantic_class == label:
                continue
            d = math.hypot(c.x - x, c.y - y)
            if d < self._mutex_dist and c.obs_count >= self._mutex_min_obs and c.obs_count > best_obs:
                best_blocker = c
                best_obs = c.obs_count

        if best_blocker is not None:
            reason = (
                "%s obs at (%.2f,%.2f) blocked by %s candidate (n=%d, d=%.2fm)"
                % (label, x, y, best_blocker.semantic_class,
                   best_blocker.obs_count,
                   math.hypot(best_blocker.x - x, best_blocker.y - y)))
            return True, reason

        return False, ""

    def _find_landmark(self, label, x, y):
        best = None
        best_score = float("inf")
        for lm in self._landmarks.values():
            if lm.semantic_class != label:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < self._merge_d:
                score = d / max(lm.observation_count, 1)
                if score < best_score:
                    best = lm
                    best_score = score
        return best

    def _find_landmark_other_class(self, label, x, y):
        """Nearest landmark of a DIFFERENT class within the evidence radius.

        The radius is `relabel_evidence_distance_m`, which defaults to the
        cross-class mutex distance rather than the (much larger) merge
        distance. That is deliberate and was measured: an observation inside
        the mutex distance of a different-class landmark is exactly what the
        old code threw away, so counting it as evidence costs the candidate
        layer nothing. Widening it to the merge distance instead starves
        candidates - over 8 acceptance runs at 1.5 m, one run absorbed 53
        chair observations while the chair candidate saw 2, and the chair was
        never promoted.

        Nearest wins, so when two different-class landmarks are both in range
        the observation is attributed to the one it most likely belongs to.
        """
        best = None
        best_d = float("inf")
        for lm in self._landmarks.values():
            if lm.semantic_class == label:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < self._evidence_d and d < best_d:
                best = lm
                best_d = d
        return best

    def _find_candidate(self, label, x, y):
        best = None
        best_d = float("inf")
        for c in self._candidates:
            if c.semantic_class != label:
                continue
            d = math.hypot(c.x - x, c.y - y)
            if d < self._cand_d and d < best_d:
                best = c
                best_d = d
        return best

    def _update_landmark(self, lm, x, y, now):
        # Position is averaged over observations of the CURRENT label only:
        # a disagreeing observation is evidence about the label, not about
        # where the object is, and letting it move the landmark would drag
        # the position towards whatever else is nearby.
        n = lm.observation_count
        lm.x = (lm.x * n + x) / (n + 1)
        lm.y = (lm.y * n + y) / (n + 1)
        lm.class_counts[lm.semantic_class] = n + 1
        lm.last_seen = now
        if lm.observation_count % 50 == 0:
            self.get_logger().info(
                "[merge_landmark] %s  n=%d  pos=(%.2f, %.2f)"
                % (lm.landmark_id, lm.observation_count, lm.x, lm.y))

    def _update_candidate(self, c, x, y, now, obs_id="-"):
        n = c.obs_count
        c.x = (c.x * n + x) / (n + 1)
        c.y = (c.y * n + y) / (n + 1)
        c.obs_count = n + 1
        c.last_seen = now
        c.feeders[obs_id] = c.feeders.get(obs_id, 0) + 1

    def _record_cross_evidence(self, lm, label, x, y, now):
        """Count a disagreeing observation against `lm`, and relabel if due."""
        lm.class_counts[label] = lm.class_counts.get(label, 0) + 1
        lm.last_seen = now
        d = math.hypot(lm.x - x, lm.y - y)
        self._note("cross_evidence", label,
                   "%s (%s) n_%s=%d d=%.2f [%s]"
                   % (lm.landmark_id, lm.semantic_class, label,
                      lm.class_counts[label], d, lm.counts_summary()))
        if not self._relabel_enabled:
            return
        new_label = should_relabel(
            lm.class_counts, lm.semantic_class, self._relabel_ratio,
            self._class_min_obs, self._min_obs)
        if new_label is not None:
            self._relabel(lm, new_label)

    def _relabel(self, lm, new_label):
        """Retire the landmark's id and re-issue it under `new_label`."""
        old_id = lm.landmark_id
        old_label = lm.semantic_class
        old_n = lm.class_counts.get(old_label, 0)
        new_n = lm.class_counts.get(new_label, 0)

        seq = self._next_seq.get(new_label, 0)
        self._next_seq[new_label] = seq + 1
        new_id = "%s_%d" % (new_label, seq)

        # The old id is retired, never reused: anything holding on to it
        # (a pending nav goal, a log line) should not silently start
        # referring to a landmark that now means something else.
        self._landmarks.pop(old_id, None)
        lm.landmark_id = new_id
        lm.semantic_class = new_label
        self._landmarks[new_id] = lm

        self.get_logger().info(
            "[relabel] relabeled %s -> %s at (%.2f, %.2f): %s %d vs %s %d"
            % (old_id, new_id, lm.x, lm.y, new_label, new_n, old_label, old_n))
        self._note("relabel", new_label,
                   "%s -> %s at (%.2f,%.2f) [%s]"
                   % (old_id, new_id, lm.x, lm.y, lm.counts_summary()))
        # Markers, marker text and landmark_objects are all rebuilt from
        # self._landmarks on the next publish tick, so colour, label text and
        # the query-visible class follow from here with no extra work.
        self._publish_markers()

    def _promote(self, c):
        seq = self._next_seq.get(c.semantic_class, 0)
        self._next_seq[c.semantic_class] = seq + 1
        lid = "%s_%d" % (c.semantic_class, seq)
        lm = Landmark(
            landmark_id=lid, semantic_class=c.semantic_class,
            x=c.x, y=c.y, last_seen=c.last_seen,
            class_counts={c.semantic_class: c.obs_count})
        self._landmarks[lid] = lm
        self._candidates.remove(c)
        self.get_logger().info(
            "[new_landmark] %s  pos=(%.2f, %.2f)  after %d obs  total=%d  feeders=%s"
            % (lid, lm.x, lm.y, c.obs_count, len(self._landmarks),
               c.feeder_summary()))

    def _cleanup_candidates(self):
        now = _time.time()
        before = len(self._candidates)
        expired = [
            c for c in self._candidates if (now - c.last_seen) >= self._cand_tout]
        self._candidates = [
            c for c in self._candidates if (now - c.last_seen) < self._cand_tout]
        removed = before - len(self._candidates)
        if removed > 0:
            self.get_logger().info("[cleanup] Removed %d stale candidates" % removed)
        # Observation only: which candidate died, how far it was from its
        # promotion threshold, and where it sat. A candidate that repeatedly
        # expires one or two observations short is the signature this
        # summary exists to expose.
        for c in expired:
            need = self._class_min_obs.get(c.semantic_class, self._min_obs)
            self._stat_expired[c.semantic_class] += 1
            self._stat_expired_max_n[c.semantic_class] = max(
                self._stat_expired_max_n[c.semantic_class], c.obs_count)
            self._stat_expired_last[c.semantic_class] = (
                "n=%d/%d (short %d) age=%.0fs pos=(%.2f,%.2f) feeders=%s"
                % (c.obs_count, need, max(0, need - c.obs_count),
                   now - c.last_seen, c.x, c.y, c.feeder_summary()))
            self.get_logger().info(
                "[cleanup] expired %s candidate %s"
                % (c.semantic_class, self._stat_expired_last[c.semantic_class]))

    # ── Gate statistics ──────────────────────────────────────────────────

    def _note(self, row, label, detail):
        """Count one event on `row` for `label` and remember its detail."""
        self._stat[row][label] += 1
        self._stat_last[row][label] = detail

    def _stat_snapshot(self):
        return (self._stat_msgs, self._stat_msgs_no_map,
                tuple(sorted(self._stat_in.items())),
                tuple((row, tuple(sorted(self._stat[row].items())))
                      for row in self.STAT_ROWS),
                tuple(sorted(self._stat_expired.items())))

    def _format_gate_summary(self, tag):
        labels = sorted(set(self._stat_in) | set(self._class_min_obs))
        elapsed = _time.time() - self._stat_t0
        head = ("[gate_summary%s] t=+%.0fs msgs=%d (before /map: %d)  in: %s"
                % (tag, elapsed, self._stat_msgs, self._stat_msgs_no_map,
                   " ".join("%s=%d" % (lb, self._stat_in.get(lb, 0))
                            for lb in labels) or "-"))
        lines = [head]
        col = max(9, max((len(lb) for lb in labels), default=9))
        lines.append("  %-15s %s  last"
                     % ("row", " ".join("%*s" % (col, lb) for lb in labels)))
        for row in self.STAT_ROWS:
            counts = self._stat[row]
            if not counts:
                continue
            # The `last` column shows the class with the most recent hit on
            # this row; per-class details are in the per-event logs of the
            # loud gates and in this row's counts for the silent ones.
            last_lb, last_txt = next(reversed(self._stat_last[row].items()))
            lines.append("  %-15s %s  %s: %s"
                         % (row,
                            " ".join("%*d" % (col, counts.get(lb, 0))
                                     for lb in labels),
                            last_lb, last_txt))
        if self._stat_expired:
            lines.append("  %-15s %s"
                         % ("expired",
                            " | ".join(
                                "%s x%d (max n=%d/%d) last %s"
                                % (lb, n, self._stat_expired_max_n[lb],
                                   self._class_min_obs.get(lb, self._min_obs),
                                   self._stat_expired_last.get(lb, ""))
                                for lb, n in sorted(self._stat_expired.items()))))
        now = _time.time()
        if self._candidates:
            lines.append("  candidates: " + " | ".join(
                "%s n=%d/%d age=%.0fs (%.2f,%.2f) [%s]"
                % (c.semantic_class, c.obs_count,
                   self._class_min_obs.get(c.semantic_class, self._min_obs),
                   now - c.last_seen, c.x, c.y, c.feeder_summary())
                for c in self._candidates))
        if self._landmarks:
            lines.append("  landmarks: " + " | ".join(
                "%s n=%d (%.2f,%.2f) counts=[%s]"
                % (lm.landmark_id, lm.observation_count, lm.x, lm.y,
                   lm.counts_summary())
                for lm in self._landmarks.values()))
        return "\n".join(lines)

    def _log_gate_summary(self, tag="", force=False):
        snap = self._stat_snapshot()
        if not force and snap == self._stat_printed:
            return
        self._stat_printed = snap
        self.get_logger().info(self._format_gate_summary(tag))

    def destroy_node(self):
        # Final table, so a run that ended between two periodic blocks still
        # leaves its totals in the log.
        try:
            self._log_gate_summary(tag=" final", force=True)
        except Exception:  # noqa: BLE001 - shutdown must not fail on a log
            pass
        super().destroy_node()

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid = 0
        for lm in self._landmarks.values():
            color = CLASS_COLORS.get(lm.semantic_class, DEFAULT_COLOR)
            s = Marker()
            s.header.stamp = stamp
            s.header.frame_id = self._frame
            s.ns = "semantic_landmarks"
            s.id = mid
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = lm.x
            s.pose.position.y = lm.y
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = self._sphere_r * 2
            s.color = color
            ma.markers.append(s)
            mid += 1
            t = Marker()
            t.header.stamp = stamp
            t.header.frame_id = self._frame
            t.ns = "semantic_landmarks_text"
            t.id = mid
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = lm.x
            t.pose.position.y = lm.y
            t.pose.position.z = self._text_z
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = "%s (n=%d)" % (lm.landmark_id, lm.observation_count)
            ma.markers.append(t)
            mid += 1
        self._pub.publish(ma)
        self._publish_landmark_objects()


    def _publish_landmark_objects(self):
        """Publish persistent landmarks as Detection3DArray for semantic_query_node."""
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        for lm in self._landmarks.values():
            det = Detection3D()
            det.header = msg.header
            det.id = lm.landmark_id
            det.bbox.center.position.x = lm.x
            det.bbox.center.position.y = lm.y
            det.bbox.center.position.z = 0.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = lm.semantic_class
            hyp.hypothesis.score = float(lm.observation_count)
            det.results.append(hyp)
            msg.detections.append(det)
        self._lm_obj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
