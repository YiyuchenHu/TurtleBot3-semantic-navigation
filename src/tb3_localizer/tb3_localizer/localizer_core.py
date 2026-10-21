#!/usr/bin/env python3
"""
localizer_core.py — Pure math for Stage-2 planar localizer.

Converts a 2D bounding-box center pixel + a LaserScan into an (x, y) position
in the robot body frame (base_link).

Pipeline per detection:
    pixel u  ─→  bearing angle θ  ─→  scan index  ─→  robust range  ─→  (x, y)

Assumptions (documented for future review):
    1. Camera optical axis is approximately aligned with robot +X.
    2. Camera horizontal center pixel corresponds to robot forward direction.
    3. Target object produces a usable LiDAR return near the same bearing.
    4. Localisation is planar (2D); no vertical angle or 3D pose estimation.
    5. LaserScan angles run [0, 2π) with 0 = robot forward (+X), increasing CCW.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class LocalizedObject:
    """Result of fusing one 2D detection with a LaserScan."""
    label: str
    confidence: float
    bearing_rad: float       # angle from robot forward (+X), CCW positive
    range_m: float           # distance to object
    x: float                 # position in base_link
    y: float                 # position in base_link
    pixel_u: float           # original bbox center x (for debug)
    scan_index: int          # central scan ray used (for debug)


class LocalizerCore:
    """Stateless fusion of 2D detections with LaserScan data."""

    def __init__(
        self,
        camera_hfov_rad: float = 1.085595,   # ~62.2° for TurtleBot3 default camera
        scan_window_half: int = 5,            # rays on each side of central ray
        min_valid_range: float = 1.0,         # reject returns closer than this (m)
        max_valid_range: float = 3.3,         # reject returns beyond this (m)
        edge_margin_px: int = 40,             # reject boxes centred this close
                                              # to the left/right image edge
    ) -> None:
        self.camera_hfov_rad = camera_hfov_rad
        self.scan_window_half = scan_window_half
        self.min_valid_range = min_valid_range
        self.max_valid_range = max_valid_range
        self.edge_margin_px = edge_margin_px

    # ------------------------------------------------------------------
    # Step 0: frame-edge boxes carry the wrong bearing
    # ------------------------------------------------------------------
    def is_edge_box(self, u: float, image_width: int) -> bool:
        """True if the box centre lies within `edge_margin_px` of a side edge.

        A box clipped by the frame edge has its centre pulled inward, so its
        bearing points beside the object and the LiDAR window at that bearing
        returns whatever stands there: measured 2026-09-03, that is how
        `trash_can` points landed on the person (u=633, conf 0.53) and grew
        candidates to n=8-10 on a target of the wrong class.
        """
        m = self.edge_margin_px
        return m > 0 and (u < m or u > image_width - m)

    # ------------------------------------------------------------------
    # Step 1: pixel → bearing
    # ------------------------------------------------------------------
    def pixel_to_bearing(self, u: float, image_width: int) -> float:
        """Convert bbox center x-pixel to bearing angle from robot forward.

        Returns bearing in radians, CCW positive:
            u < W/2  →  positive bearing (object is to the LEFT)
            u > W/2  →  negative bearing (object is to the RIGHT)
            u = W/2  →  0 (dead ahead)

        The sign convention follows ROS REP 103 (right-hand rule, +X forward,
        +Y left).  In image coordinates +u is rightward, so the sign flips.
        """
        half_w = image_width / 2.0
        normalised = (u - half_w) / half_w          # −1 (left edge) … +1 (right edge)
        bearing = -normalised * (self.camera_hfov_rad / 2.0)
        return bearing

    # ------------------------------------------------------------------
    # Step 2: bearing → scan index
    # ------------------------------------------------------------------
    def bearing_to_scan_index(
        self,
        bearing: float,
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        num_ranges: int,
    ) -> Optional[int]:
        """Map a bearing (rad, CCW from +X) to LaserScan array index.

        The scan convention for TurtleBot3 is [0, 2π) with 0 = forward.
        A bearing of e.g. −0.3 rad (slightly right) maps to scan_angle ≈ 5.98 rad.
        """
        scan_angle = bearing % (2.0 * math.pi)

        index = round((scan_angle - angle_min) / angle_increment)
        if index < 0 or index >= num_ranges:
            return None
        return index

    # ------------------------------------------------------------------
    # Step 3: robust range from scan window
    # ------------------------------------------------------------------
    def robust_range(
        self, ranges: list[float], center_index: int
    ) -> Optional[float]:
        """Return the median of valid ranges in a window around *center_index*.

        Invalid values (nan, inf, out-of-bounds) are discarded.  Returns None if
        no valid range remains — meaning the object has no usable LiDAR return.
        """
        n = len(ranges)
        lo = max(0, center_index - self.scan_window_half)
        hi = min(n, center_index + self.scan_window_half + 1)

        valid: list[float] = []
        for i in range(lo, hi):
            r = ranges[i]
            if math.isfinite(r) and self.min_valid_range <= r <= self.max_valid_range:
                valid.append(r)

        if not valid:
            return None

        valid.sort()
        mid = len(valid) // 2
        return valid[mid]

    # ------------------------------------------------------------------
    # Step 4: bearing + range → (x, y) in base_link
    # ------------------------------------------------------------------
    @staticmethod
    def polar_to_cartesian(bearing: float, range_m: float) -> tuple[float, float]:
        """Convert (bearing, range) to (x, y) in base_link.

        x = forward, y = left (ROS REP 103).
        """
        x = range_m * math.cos(bearing)
        y = range_m * math.sin(bearing)
        return x, y

    # ------------------------------------------------------------------
    # Full pipeline: one detection → one LocalizedObject (or None)
    # ------------------------------------------------------------------
    def localize(
        self,
        label: str,
        confidence: float,
        bbox_center_x: float,
        image_width: int,
        scan_ranges: list[float],
        scan_angle_min: float,
        scan_angle_max: float,
        scan_angle_increment: float,
    ) -> Optional[LocalizedObject]:
        """Run the full pixel → bearing → scan → (x,y) pipeline for one detection."""

        if image_width <= 0:
            return None

        if self.is_edge_box(bbox_center_x, image_width):
            return None

        bearing = self.pixel_to_bearing(bbox_center_x, image_width)

        idx = self.bearing_to_scan_index(
            bearing,
            scan_angle_min,
            scan_angle_max,
            scan_angle_increment,
            len(scan_ranges),
        )
        if idx is None:
            return None

        range_m = self.robust_range(scan_ranges, idx)
        if range_m is None:
            return None

        x, y = self.polar_to_cartesian(bearing, range_m)

        return LocalizedObject(
            label=label,
            confidence=confidence,
            bearing_rad=bearing,
            range_m=range_m,
            x=x,
            y=y,
            pixel_u=bbox_center_x,
            scan_index=idx,
        )
