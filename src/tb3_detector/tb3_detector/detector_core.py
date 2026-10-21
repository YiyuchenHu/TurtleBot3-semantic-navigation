#!/usr/bin/env python3
"""
detector_core.py  —  Stage-1 perception: YOLO26 inference wrapper.

This is the full implementation of load() and infer(): the fine-tuned
YOLO26n weights (person / trash_can / chair) run on every camera frame and
the whole semantic-navigation stack works end to end.

Responsibilities of this class:
  - Load a YOLO26 model from a configurable local path.
  - Run inference on a BGR numpy image (from cv_bridge).
  - Return a list of Detection dicts (format below).

NOT responsible for:
  - Coordinate projection to 3D (→ tb3_localizer, provided)
  - Maintaining object history  (→ tb3_memory,    provided)
  - Answering semantic queries  (→ tb3_query,     provided)
  - Sending Nav2 goals          (→ tb3_nav_adapter / tb3_coordinator, provided)

============================================================================
OUTPUT CONTRACT — do not change key names; the provided localizer and the
node wrapper (detector_node.py) rely on them:

    {
        "label":      str,          # detector class name: "person", "trash_can", "chair"
        "conf":       float,        # confidence, 0.0 – 1.0
        "bbox_xyxy":  [x1, y1, x2, y2],   # pixels (float), see convention below
        "track_id":   int | None,   # None unless you enable tracking
    }

Pixel-coordinate convention (standard image coordinates):
    - origin (0, 0) is the TOP-LEFT corner of the image
    - x grows to the RIGHT, y grows DOWN
    - (x1, y1) = top-left corner of the box, (x2, y2) = bottom-right corner
    - values are in pixels of the ORIGINAL /camera/image_raw resolution —
      if you resize the image for inference, scale the boxes back!
      (ultralytics already returns boxes in original-image pixels.)
    The provided localizer uses the bbox centre x = (x1+x2)/2 to compute a
    bearing through the camera FOV, so a wrong x coordinate sends the robot
    in a wrong direction.
============================================================================
"""

from __future__ import annotations
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Optional import — graceful failure if ultralytics is not installed.
# Keeping this guarded means the package still *builds* without the Python
# dependencies; load() is where the missing dependency becomes a hard error.
#
#     pip install 'ultralytics==8.4.31'   # pinned; yolo26n needs >= 8.4.x
#     pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
# ---------------------------------------------------------------------------
try:
    from ultralytics import YOLO as _UltralyticsYOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _UltralyticsYOLO = None
    _ULTRALYTICS_AVAILABLE = False
    logger.warning(
        "ultralytics not found. Install with:  pip install 'ultralytics==8.4.31'\n"
        "detector_core will raise RuntimeError on load() until then."
    )


# Keys every downstream consumer may rely on:
DETECTION_KEYS = ("label", "conf", "bbox_xyxy", "track_id")


class DetectorCore:
    """
    Thin wrapper around a YOLO26 model.

    Usage::

        core = DetectorCore(model_path="models/tb3det_yolo26n.pt", conf_threshold=0.40)
        core.load()                          # loads weights once at startup
        detections = core.infer(bgr_image)   # list of dicts (see module docstring)

    Parameters
    ----------
    model_path : str | Path
        Absolute or relative path to the YOLO26 .pt weights file.
    conf_threshold : float
        Minimum confidence to include a detection (0.0 – 1.0).
        The shipped config uses 0.40 for the fine-tuned tb3det_yolo26n weights
        (threshold sweep in config/detector.yaml). Tune it there, not here.
    class_filter : list[str] | None
        If given, only return detections whose label is in this list.
        None means return all detected classes.
        IMPORTANT: entries are *detector labels* as the weights emit them
        ("person", "trash_can", "chair" for the shipped weights; the COCO
        weights say "traffic light" for the trash can), NOT the task-level
        semantic names — see the mapping note below.
    device : str
        Torch device string, e.g. "cpu", "cuda:0".
    enable_tracking : bool
        If True, use model.track() instead of model.predict() (ByteTrack).
        Optional — the course task only requires plain per-frame detection.

    ------------------------------------------------------------------------
    Detector label → task label mapping (important!)
    ------------------------------------------------------------------------
    The mapping used by the rest of the stack lives in
    src/tb3_bringup/config/semantic_targets.yaml:

        task name    detector_label     Gazebo model
        ---------    --------------     ------------
        person   ←   "person"           person_standing
        trash_can ←  "trash_can"        first_2015_trash_can
        chair    ←   "chair"            VisitorChair

    With the shipped fine-tuned weights the two columns agree. They did not
    with the COCO weights (no trash-can class; yolo26n called that model a
    *traffic light*), and that is why detector_label and semantic_name are
    separate fields: DetectorCore reports whatever the network says, and the
    provided downstream nodes translate it using semantic_targets.yaml. Do not
    rename labels inside infer().
    """

    def __init__(
        self,
        model_path: str | Path,
        conf_threshold: float = 0.35,
        class_filter: list[str] | None = None,
        device: str = "cpu",
        enable_tracking: bool = False,
    ) -> None:
        self.model_path = Path(model_path)
        self.conf_threshold = float(conf_threshold)
        self.class_filter = set(class_filter) if class_filter else None
        self.device = device
        self.enable_tracking = enable_tracking

        self._model: Any = None   # set by load()

    # ------------------------------------------------------------------
    def load(self) -> None:
        """
        Load model weights.  Called once by detector_node at startup.

        Raises RuntimeError if ultralytics is missing, FileNotFoundError if
        the weights file is absent — both with a readable message.

        The fine-tuned weights (models/tb3det_yolo26n.pt) ship with the repo;
        after a fresh checkout run colcon build --symlink-install
        --packages-select tb3_detector so the file is copied into the install
        tree, or pass an absolute model_path. The COCO comparison weights are
        a local download (see src/tb3_detector/models/README.md).
        """
        if not _ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "ultralytics package is not installed. "
                "Run:  pip install 'ultralytics==8.4.31'"
            )
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Model file not found: {self.model_path}\n"
                "► See INSTRUCTIONS.md → model download step."
            )

        logger.info("Loading YOLO26 model from %s on device=%s", self.model_path, self.device)
        self._model = _UltralyticsYOLO(str(self.model_path))
        self._model.to(self.device)
        logger.info("Model loaded. Classes: %s", list(self._model.names.values()))

    # ------------------------------------------------------------------
    def infer(self, bgr_image) -> list[dict]:
        """
        Run inference on a single BGR uint8 numpy array (shape H×W×3).

        Returns a (possibly empty) list of detection dicts, format specified
        in the module docstring. Never returns None.

        ultralytics accepts BGR numpy arrays directly; no colour conversion or
        resizing is needed, and boxes come back in original-image pixels.
        """
        if self._model is None:
            raise RuntimeError("DetectorCore.load() has not been called yet.")

        if self.enable_tracking:
            results = self._model.track(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                persist=True,
                verbose=False,
            )
        else:
            results = self._model.predict(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )

        detections: list[dict] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            names = result.names  # int -> str

            for i in range(len(boxes)):
                label = names[int(boxes.cls[i].item())]
                if self.class_filter and label not in self.class_filter:
                    continue

                conf = float(boxes.conf[i].item())
                xyxy = boxes.xyxy[i].tolist()  # [x1, y1, x2, y2]

                track_id = None
                if self.enable_tracking and boxes.id is not None:
                    track_id = int(boxes.id[i].item())

                detections.append(
                    {
                        "label":     label,
                        "conf":      conf,
                        "bbox_xyxy": xyxy,
                        "track_id":  track_id,
                    }
                )

        return detections

    # ------------------------------------------------------------------
    @property
    def is_loaded(self) -> bool:
        return self._model is not None
