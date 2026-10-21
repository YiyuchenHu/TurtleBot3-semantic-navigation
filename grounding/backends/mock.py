"""GPU-free stand-in backend: resolves color terms in the query with an
HSV mask and returns bounding boxes of the largest matching regions.

Good enough to demo "sofa with warm color" end-to-end without the real
model. Queries with no recognized color term return no boxes.
"""

from __future__ import annotations

import cv2
import numpy as np

# OpenCV hue range is [0, 179].
_HUE_RANGES: dict[str, list[tuple[int, int]]] = {
    "red":    [(0, 10), (170, 179)],
    "orange": [(11, 25)],
    "yellow": [(26, 34)],
    "green":  [(35, 85)],
    "cyan":   [(86, 100)],
    "blue":   [(101, 130)],
    "purple": [(131, 155)],
    "violet": [(131, 155)],
    "pink":   [(156, 169)],
}

_WARM_TERMS = ("red", "orange", "yellow", "pink")
_COOL_TERMS = ("green", "cyan", "blue", "purple")

_MIN_SATURATION = 70
_MIN_VALUE = 50
_MAX_REGIONS = 3
_MIN_REGION_FRAC = 0.001


class MockColorBackend:

    def __init__(self, **_kwargs) -> None:
        pass

    def load(self) -> None:
        pass

    def info(self) -> dict:
        return {"backend": "mock", "device": "cpu", "model": "hsv-color-heuristic"}

    def detect(self, image_bytes: bytes, categories: list[str],
               options: dict | None = None) -> list[dict]:
        """Category-list detection: runs the color heuristic per category
        and labels each hit with its category. Categories without a
        recognized color term contribute no boxes (mirrors the real
        model's honest-negative behavior). `options` (generation options
        of the real backend) is accepted and ignored."""
        detections: list[dict] = []
        for cat in categories:
            for box in self.ground(image_bytes, cat):
                box["label"] = cat
                detections.append(box)
        return detections

    def ground(self, image_bytes: bytes, query: str,
               options: dict | None = None) -> list[dict]:
        bgr = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("could not decode image bytes")

        ranges = self._ranges_for_query(query)
        if not ranges:
            return []

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, (lo, _MIN_SATURATION, _MIN_VALUE), (hi, 255, 255))

        # Close small gaps so one object forms one component.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        h, w = mask.shape
        min_px = max(int(_MIN_REGION_FRAC * w * h), 16)

        regions = []
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if area < min_px:
                continue
            density = float(area) / float(max(bw * bh, 1))
            regions.append({
                "bbox_xyxy": [float(x), float(y), float(x + bw), float(y + bh)],
                "score": round(min(1.0, density), 4),
                "label": "",
                "area": int(area),
            })

        regions.sort(key=lambda r: -r["area"])
        return [
            {k: r[k] for k in ("bbox_xyxy", "score", "label")}
            for r in regions[:_MAX_REGIONS]
        ]

    @staticmethod
    def _ranges_for_query(query: str) -> list[tuple[int, int]]:
        q = (query or "").lower()
        terms: set[str] = set()
        if "warm" in q:
            terms.update(_WARM_TERMS)
        if "cool" in q or "cold" in q:
            terms.update(_COOL_TERMS)
        for name in _HUE_RANGES:
            if name in q:
                terms.add(name)
        ranges: list[tuple[int, int]] = []
        for t in terms:
            ranges.extend(_HUE_RANGES[t])
        return ranges
