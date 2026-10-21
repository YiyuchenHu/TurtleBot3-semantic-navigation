#!/usr/bin/env python3
"""
detector_node.py  —  ROS 2 node wrapper for Stage-1 YOLO26 detection.

──────────────────────────────────────────────────────────────────────────
The node:
  - subscribes to the camera,
  - calls DetectorCore.infer() on every frame,
  - converts the returned dicts to vision_msgs/Detection2DArray,
  - publishes a debug image with the boxes drawn on top.
────────────────────────────────────────────────────────────────────────────

Subscribed topics
-----------------
  /camera/image_raw          sensor_msgs/Image      (raw BGR / RGB camera)
  /camera/camera_info        sensor_msgs/CameraInfo (cached; not needed for 2D detection)

Published topics
----------------
  ~/detections               vision_msgs/Detection2DArray
      Contains one Detection2D per detected object:
        - results[0].hypothesis.class_id  = class label (str)
        - results[0].hypothesis.score     = confidence (float 0-1)
        - bbox.center.position.x / .y     = box centre (pixels)
        - bbox.size_x / size_y            = box size   (pixels)
        - id                              = track_id or "" if none

  ~/debug_image              sensor_msgs/Image   (only if ~publish_debug_image: true)
      Camera image with bounding-box overlays drawn from the detections.

Parameters
----------
  model_path              str   Path to the .pt weights (shipped: tb3det_yolo26n.pt)
  conf_threshold          float 0.40 in the shipped config (see detector.yaml)
  device                  str   "cpu" | "cuda:0"
  class_filter            list  [""] means all classes; detector labels
                                (shipped: "person", "trash_can", "chair")
  enable_tracking         bool  false
  publish_debug_image     bool  true
  image_topic             str   "/camera/image_raw"
  camera_info_topic       str   "/camera/camera_info"
  detections_topic        str   "~/detections"
  debug_image_topic       str   "~/debug_image"
  queue_size              int   5
"""

from __future__ import annotations
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import cv2

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(
        "cv_bridge not found. Install ros-humble-cv-bridge.\n"
        f"Original error: {e}"
    )

# Local detector logic (same package) — the YOLO26 wrapper.
from tb3_detector.detector_core import DetectorCore


# ---------------------------------------------------------------------------
# Helpers — they define the exact wire format of the detections
# ---------------------------------------------------------------------------

def _make_detection2d_array(detections: list[dict], stamp, frame_id: str) -> Detection2DArray:
    """Convert list of DetectorCore dicts → vision_msgs/Detection2DArray."""
    msg = Detection2DArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    for d in detections:
        det = Detection2D()
        det.header.stamp = stamp
        det.header.frame_id = frame_id

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = d["label"]
        hyp.hypothesis.score = float(d["conf"])
        det.results.append(hyp)

        x1, y1, x2, y2 = d["bbox_xyxy"]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = x2 - x1
        h = y2 - y1

        det.bbox.center.position.x = cx
        det.bbox.center.position.y = cy
        det.bbox.size_x = w
        det.bbox.size_y = h

        # track_id as string id field (used by memory when tracking is enabled)
        if d.get("track_id") is not None:
            det.id = str(d["track_id"])

        msg.detections.append(det)

    return msg


def _draw_detections(bgr_image, detections: list[dict]):
    """Draw bounding boxes + labels on a copy of bgr_image (for debug only)."""
    vis = bgr_image.copy()
    for d in detections:
        x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
        label = d["label"]
        conf = d["conf"]
        tid = d.get("track_id")
        tag = f"{label} {conf:.2f}"
        if tid is not None:
            tag += f" #{tid}"
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 50), 2)
        cv2.putText(vis, tag, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 50), 2)
    return vis


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DetectorNode(Node):
    """
    Stage-1 detector node.

    Lifecycle:
        __init__  → declare params, load model, create pubs/subs.
    The node then runs fully via subscription callbacks; no timer loop needed.
    """

    def __init__(self) -> None:
        super().__init__("detector_node")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("class_filter", [""])          # empty list = all classes
        self.declare_parameter("enable_tracking", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("detections_topic", "~/detections")
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("queue_size", 5)

        model_path_raw = self.get_parameter("model_path").get_parameter_value().string_value
        conf           = self.get_parameter("conf_threshold").get_parameter_value().double_value
        device         = self.get_parameter("device").get_parameter_value().string_value

        # rclpy Humble cannot infer a type from a bare `[]` in YAML, which leaves the
        # parameter as PARAMETER_NOT_SET even though declare_parameter provided a default.
        # Guard with a fallback so `class_filter: []` and `class_filter: [""]` both mean
        # "no filter / accept all classes".
        try:
            raw_filter = self.get_parameter("class_filter").get_parameter_value().string_array_value
        except Exception:
            raw_filter = []
        tracking       = self.get_parameter("enable_tracking").get_parameter_value().bool_value
        self._pub_debug = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        image_topic    = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic     = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        det_topic      = self.get_parameter("detections_topic").get_parameter_value().string_value
        dbg_topic      = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        queue          = self.get_parameter("queue_size").get_parameter_value().integer_value

        # Resolve model path: allow relative (to share dir) or absolute
        model_path = self._resolve_model_path(model_path_raw)

        # class_filter: [""] means "no filter"
        class_filter = [c for c in raw_filter if c.strip()] or None

        # ── Inference core ─────────────────────────────────────────────────
        self._core = DetectorCore(
            model_path=model_path,
            conf_threshold=conf,
            class_filter=class_filter,
            device=device,
            enable_tracking=tracking,
        )
        try:
            self._core.load()
        except Exception as exc:
            self.get_logger().error("DetectorCore failed to load: %s" % exc)
            self.get_logger().error(
                "► Check README → Setup by platform, steps 2 and 4: are "
                "ultralytics/torch installed, and "
                "was tb3_detector rebuilt so models/tb3det_yolo26n.pt reached install/?"
            )
            raise

        # ── cv_bridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── QoS ────────────────────────────────────────────────────────────
        # Camera images from Gazebo are published BEST_EFFORT — subscribing
        # RELIABLE would silently match nothing. Keep these QoS settings.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=queue,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=queue,
        )

        # ── Subscriptions ──────────────────────────────────────────────────
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, sensor_qos
        )
        self._latest_camera_info: CameraInfo | None = None

        # ── Publishers ─────────────────────────────────────────────────────
        self._det_pub = self.create_publisher(Detection2DArray, det_topic, reliable_qos)
        if self._pub_debug:
            self._dbg_pub = self.create_publisher(Image, dbg_topic, reliable_qos)
        else:
            self._dbg_pub = None

        self.get_logger().info(
            "detector_node ready. "
            "image_topic=%s  class_filter=%s  conf=%.2f  device=%s  tracking=%s"
            % (image_topic, class_filter, conf, device, tracking)
        )

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Cache latest camera info (kept for future 3-D projection work)."""
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        """Main callback: convert → infer → publish."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning("cv_bridge conversion failed: %s" % e)
            return

        # ── Run inference ───────────────────────────────────────────────
        try:
            detections = self._core.infer(bgr)
        except Exception as e:
            self.get_logger().error("Inference error: %s" % e)
            return

        stamp = msg.header.stamp
        frame = msg.header.frame_id

        # ── Publish detections ──────────────────────────────────────────
        det_msg = _make_detection2d_array(detections, stamp, frame)
        self._det_pub.publish(det_msg)

        if detections:
            labels = [d["label"] for d in detections]
            self.get_logger().debug("Detected: %s" % labels)

        # ── Publish debug image ─────────────────────────────────────────
        if self._dbg_pub is not None:
            vis = _draw_detections(bgr, detections) if detections else bgr
            try:
                dbg_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
                dbg_msg.header = msg.header
                self._dbg_pub.publish(dbg_msg)
            except Exception as e:
                self.get_logger().warning("Debug image publish failed: %s" % e)

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _resolve_model_path(self, raw: str) -> Path:
        """
        Resolve model path.
        1. If absolute and exists → use as-is.
        2. If relative → try relative to share/tb3_detector/models/.
        3. Else → return raw (DetectorCore decides how to fail).
        """
        p = Path(raw)
        if p.is_absolute():
            return p
        # Try share directory (after colcon install)
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("tb3_detector"))
            candidate = share / "models" / p
            if candidate.is_file():
                return candidate
        except Exception:
            pass
        # Fall back to the raw string (DetectorCore.load should handle it)
        return p


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
