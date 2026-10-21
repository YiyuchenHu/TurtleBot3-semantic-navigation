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
        self.declare_parameter("min_valid_range", 1.0)
        self.declare_parameter("max_valid_range", 3.3)
        self.declare_parameter("edge_margin_px", 40)
        self.declare_parameter("edge_drop_summary_sec", 30.0)
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

        # ── State: latest scan and image width ────────────────────────────
        self._latest_scan: LaserScan | None = None
        self._image_width: int = 0

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

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _image_cb(self, msg: Image) -> None:
        if self._image_width == 0:
            self._image_width = msg.width
            self.get_logger().info("Image width learned: %d px" % msg.width)

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
