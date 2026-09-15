#!/usr/bin/env python3
"""
localizer_node.py — Stage-2 planar localizer ROS 2 node.

Fuses 2D object detections (from tb3_detector) with LiDAR range data to
produce per-object PointStamped positions in the robot body frame.

Subscribed topics
-----------------
  /detector_node/detections   vision_msgs/Detection2DArray
  /scan                       sensor_msgs/LaserScan
  /camera/image_raw           sensor_msgs/Image   (only to learn image width)

Published topics
----------------
  ~/object_points             geometry_msgs/PointStamped   (one msg per localised detection)
  ~/localized_objects         vision_msgs/Detection3DArray (batch per callback, for Stage-3 memory)

Parameters  (see config/localizer.yaml)
----------
  camera_hfov_deg             float   Camera horizontal FOV in degrees
  scan_window_half            int     Half-width of scan averaging window
  min_valid_range             float   Minimum acceptable LiDAR range (m)
  max_valid_range             float   Maximum acceptable LiDAR range (m)
  detections_topic            str     Detection input topic
  scan_topic                  str     LaserScan topic
  image_topic                 str     RGB image topic (for width only)
  output_topic                str     Output PointStamped topic
  output_frame                str     Frame for published points
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PointStamped, Pose
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    BoundingBox3D,
    ObjectHypothesisWithPose,
)

from tb3_localizer.localizer_core import LocalizerCore


class LocalizerNode(Node):

    def __init__(self) -> None:
        super().__init__("localizer_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("camera_hfov_deg", 62.2)
        self.declare_parameter("scan_window_half", 5)
        self.declare_parameter("min_valid_range", 0.12)
        self.declare_parameter("max_valid_range", 8.0)
        self.declare_parameter("edge_margin_px", 40)
        self.declare_parameter("edge_drop_summary_sec", 30.0)
        # ── bbox height ↔ ray distance consistency gate (R05) ──────────
        # The scan ray is cast at the bbox CENTRE COLUMN, so a detection
        # of a FAR object whose column crosses a NEARER obstacle gets the
        # nearer range and its observation lands on the wrong object
        # (R04 §4.4; R05 mapping smoke: three "person wearing a blue
        # shirt" ghosts and one cone ghost, all from far boxes localized
        # onto near objects — the occlusion filter only helps when the
        # occluder is DETECTED in the same frame). Gate: a real object at
        # ray distance d must subtend ≈ f·H/d pixels; a bbox SHORTER than
        # (expected / tolerance) belongs to something farther than the
        # ray hit, so the pairing is wrong and the observation is
        # dropped. Deliberately ONE-SIDED: a bbox TALLER than expected is
        # just frame-clipping at close range (a 1.75 m person overflows
        # the 480 px image inside ~2 m) and stays accepted.
        self.declare_parameter("height_gate_enabled", True)
        self.declare_parameter("height_gate_tolerance", 2.0)
        # "label:metres[:tolerance]". Matching: exact label first, else any
        # label containing "person" uses the person prior (covers the
        # shirt color expressions and any injected person search term).
        # Squat classes carry their own TIGHTER tolerance: their absolute
        # pixel heights are small, so the default 2.0 leaves the gate
        # toothless exactly where the R05 smoke-3 cone ghost slipped
        # through (far cone 88 px vs 84 px required on a 1.1 m ray).
        # Squat classes (prior <= 0.8 m) are also gated TWO-SIDED: a box
        # much TALLER than expected is a person force-matched into the
        # class (they never overflow the frame at gate-relevant ranges,
        # so the close-range clipping argument does not apply to them).
        self.declare_parameter("height_priors", [
            "person:1.75", "bookshelf:1.2:1.5", "trash can:0.6:1.5",
            "traffic cone:0.9:1.4", "concrete barrier:1.1"])
        self.declare_parameter("detections_topic", "/detector_node/detections")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "~/object_points")
        self.declare_parameter("output_frame", "base_link")

        hfov_deg         = self.get_parameter("camera_hfov_deg").value
        scan_window_half = self.get_parameter("scan_window_half").value
        min_range        = self.get_parameter("min_valid_range").value
        max_range        = self.get_parameter("max_valid_range").value
        edge_margin      = int(self.get_parameter("edge_margin_px").value)
        summary_sec      = float(self.get_parameter("edge_drop_summary_sec").value)
        self._hgate_on   = bool(self.get_parameter("height_gate_enabled").value)
        self._hgate_tol  = float(self.get_parameter("height_gate_tolerance").value)
        # label -> (height_m, tolerance)
        self._height_priors: dict[str, tuple[float, float]] = {}
        for entry in self.get_parameter("height_priors").value:
            parts = str(entry).split(":")
            if len(parts) >= 2:
                tol = float(parts[2]) if len(parts) >= 3 else self._hgate_tol
                self._height_priors[parts[0].strip().lower()] = \
                    (float(parts[1]), tol)
        det_topic        = self.get_parameter("detections_topic").value
        scan_topic       = self.get_parameter("scan_topic").value
        image_topic      = self.get_parameter("image_topic").value
        out_topic        = self.get_parameter("output_topic").value
        self._out_frame  = self.get_parameter("output_frame").value

        # ── Core math ─────────────────────────────────────────────────────
        self._core = LocalizerCore(
            camera_hfov_rad=math.radians(hfov_deg),
            scan_window_half=scan_window_half,
            min_valid_range=min_range,
            max_valid_range=max_range,
            edge_margin_px=edge_margin,
        )

        # ── State: latest scan and image size ─────────────────────────────
        self._latest_scan: LaserScan | None = None
        self._image_width: int = 0
        self._image_height: int = 0
        self._hgate_dropped: dict[str, int] = {}

        # ── Edge-box bookkeeping (observability only) ─────────────────────
        # Per class: detections seen / dropped as frame-edge boxes, printed
        # as one throttled line so the share is visible in the log.
        self._edge_seen: dict[str, int] = {}
        self._edge_dropped: dict[str, int] = {}
        self._edge_printed: tuple = ()
        if summary_sec > 0.0:
            self.create_timer(summary_sec, self._log_edge_summary)

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, sensor_qos)
        self.create_subscription(
            Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(
            Detection2DArray, det_topic, self._detections_cb, reliable_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self._point_pub = self.create_publisher(PointStamped, out_topic, reliable_qos)
        self._det3d_pub = self.create_publisher(
            Detection3DArray, "~/localized_objects", reliable_qos
        )

        self.get_logger().info(
            "LocalizerNode ready  "
            "hfov=%.1f°  scan_window=%d  range=[%.2f, %.2f]  edge_margin=%dpx  frame=%s"
            % (hfov_deg, scan_window_half, min_range, max_range, edge_margin,
               self._out_frame)
        )

    def _log_edge_summary(self) -> None:
        snap = tuple(sorted(self._edge_seen.items())) + tuple(sorted(self._edge_dropped.items()))
        if not self._edge_seen or snap == self._edge_printed:
            return
        self._edge_printed = snap
        parts = []
        for label in sorted(self._edge_seen):
            n = self._edge_seen[label]
            d = self._edge_dropped.get(label, 0)
            parts.append("%s %d/%d (%.1f%%)" % (label, d, n, 100.0 * d / max(1, n)))
        self.get_logger().info("[edge_drop] dropped/seen per class: " + "  ".join(parts))
        if self._hgate_dropped:
            self.get_logger().info(
                "[height_gate] dropped per class: " + "  ".join(
                    "%s %d" % kv for kv in sorted(self._hgate_dropped.items())))

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _image_cb(self, msg: Image) -> None:
        if self._image_width == 0:
            self._image_width = msg.width
            self._image_height = msg.height
            self.get_logger().info(
                "Image size learned: %dx%d px" % (msg.width, msg.height))

    def _height_prior(self, label: str) -> tuple[float, float] | None:
        prior = self._height_priors.get(label.lower())
        if prior is None and "person" in label.lower():
            prior = self._height_priors.get("person")
        return prior

    # Classes at or below this height get the two-sided check (a much
    # taller box is a person force-matched into the class).
    _SQUAT_MAX_H = 0.8

    def _height_gate_ok(self, label: str, bbox_h_px: float,
                        range_m: float) -> bool:
        """False when the bbox height is inconsistent with an object of
        this class at the ray distance (see parameter block). Too short =
        the box belongs to something farther than the ray hit (all
        classes); too tall = a taller object force-matched into a squat
        class (squat classes only)."""
        if not self._hgate_on or bbox_h_px <= 0.0 or range_m <= 0.0:
            return True
        prior = self._height_prior(label)
        if prior is None:
            return True
        height_m, tol = prior
        focal_px = (self._image_width / 2.0) / math.tan(
            self._core.camera_hfov_rad / 2.0)
        expected_px = focal_px * height_m / range_m
        required_px = expected_px / tol
        if self._image_height > 0:
            # Never require more than the frame can show (a close object's
            # box clips at the image height). The cap applies to the
            # REQUIREMENT, after tolerance — capping the expectation first
            # would let tolerance shrink the requirement below what a real
            # clipped box always shows.
            required_px = min(required_px, 0.9 * self._image_height)
        if bbox_h_px < required_px:
            return False
        if height_m <= self._SQUAT_MAX_H and bbox_h_px > expected_px * tol:
            return False
        return True

    def _detections_cb(self, msg: Detection2DArray) -> None:
        scan = self._latest_scan
        if scan is None:
            return
        if self._image_width <= 0:
            return
        if not msg.detections:
            return

        det3d_array = Detection3DArray()
        det3d_array.header.stamp = msg.header.stamp
        det3d_array.header.frame_id = self._out_frame

        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id
            conf  = det.results[0].hypothesis.score
            u     = det.bbox.center.position.x

            self._edge_seen[label] = self._edge_seen.get(label, 0) + 1
            if self._core.is_edge_box(u, self._image_width):
                self._edge_dropped[label] = self._edge_dropped.get(label, 0) + 1
                self.get_logger().debug(
                    "[edge_drop] '%s' u=%.0f within %dpx of the frame edge"
                    % (label, u, self._core.edge_margin_px))
                continue

            result = self._core.localize(
                label=label,
                confidence=conf,
                bbox_center_x=u,
                image_width=self._image_width,
                scan_ranges=list(scan.ranges),
                scan_angle_min=scan.angle_min,
                scan_angle_max=scan.angle_max,
                scan_angle_increment=scan.angle_increment,
            )

            if result is None:
                self.get_logger().debug(
                    "No valid range for '%s' (u=%.0f, bearing=%.2f rad)"
                    % (label, u, self._core.pixel_to_bearing(u, self._image_width))
                )
                continue

            if not self._height_gate_ok(label, det.bbox.size_y,
                                        result.range_m):
                self._hgate_dropped[label] = \
                    self._hgate_dropped.get(label, 0) + 1
                self.get_logger().info(
                    "[height_gate] dropped '%s': bbox %.0fpx vs %.2fm ray "
                    "(short = far box on near obstacle; tall = bigger "
                    "object force-matched into a squat class)"
                    % (label, det.bbox.size_y, result.range_m),
                    throttle_duration_sec=10.0)
                continue

            # ── PointStamped (backward compat) ────────────────────────
            pt = PointStamped()
            pt.header.stamp = msg.header.stamp
            pt.header.frame_id = self._out_frame
            pt.point.x = result.x
            pt.point.y = result.y
            pt.point.z = 0.0
            self._point_pub.publish(pt)

            # ── Detection3D (for Stage-3 memory) ─────────────────────
            d3 = Detection3D()
            d3.header.stamp = msg.header.stamp
            d3.header.frame_id = self._out_frame

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = result.label
            hyp.hypothesis.score = float(result.confidence)
            d3.results.append(hyp)

            d3.bbox.center.position.x = result.x
            d3.bbox.center.position.y = result.y
            d3.bbox.center.position.z = 0.0

            det3d_array.detections.append(d3)

            self.get_logger().info(
                "[%s] conf=%.2f  u=%.0f  bearing=%.1f°  range=%.2fm  "
                "→ base_link (%.2f, %.2f)"
                % (
                    result.label,
                    result.confidence,
                    result.pixel_u,
                    math.degrees(result.bearing_rad),
                    result.range_m,
                    result.x,
                    result.y,
                )
            )

        if det3d_array.detections:
            self._det3d_pub.publish(det3d_array)

        # TODO: transform points from base_link to map frame using TF


def main(args=None):
    rclpy.init(args=args)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
