"""Parse LocateAnything box tokens into pixel-space boxes.

The model emits boxes as `<box><x1><y1><x2><y2></box>` with coordinates
normalized to [0, 1000]; see https://huggingface.co/nvidia/LocateAnything-3B.
In detect mode (category-list prompt) boxes are grouped under `<ref>label</ref>`
tags; a group with no instances is the literal `<box>None</box>`.
Importable without torch.
"""

from __future__ import annotations

import re

_BOX_RE = re.compile(r"<box><(\d+)><(\d+)><(\d+)><(\d+)></box>")
_REF_SPLIT_RE = re.compile(r"<ref>(.*?)</ref>", re.DOTALL)


def parse_boxes(text: str, image_width: int, image_height: int) -> list[dict]:
    """Return [{"bbox_xyxy": [x1, y1, x2, y2], "score": 1.0, "label": ""}, ...].

    The model emits no per-box confidence, so score is a constant 1.0;
    downstream ranking should rely on IoU against a reference box instead.
    Coordinates are clamped to the image; degenerate boxes are dropped.
    """
    boxes = []
    for m in _BOX_RE.finditer(text or ""):
        x1, y1, x2, y2 = (int(g) / 1000.0 for g in m.groups())
        px1 = _clamp(x1 * image_width, 0, image_width)
        py1 = _clamp(y1 * image_height, 0, image_height)
        px2 = _clamp(x2 * image_width, 0, image_width)
        py2 = _clamp(y2 * image_height, 0, image_height)
        if px2 - px1 < 1.0 or py2 - py1 < 1.0:
            continue
        boxes.append({"bbox_xyxy": [px1, py1, px2, py2], "score": 1.0, "label": ""})
    return boxes


def parse_labeled_boxes(text: str, image_width: int, image_height: int,
                        fallback_label: str = "") -> list[dict]:
    """Parse `<ref>label</ref><box>..</box>..` grouped output (detect mode).

    Boxes following a `<ref>` tag carry that tag's text as their label,
    up to the next `<ref>`. Boxes before any `<ref>`, or under an empty
    one, get `fallback_label` (referring mode emits prompt-echo `<ref>`
    fragments, so callers pass the query phrase there). `<box>None</box>`
    (the model's honest "no instances") yields no boxes by construction —
    it never matches the numeric box pattern.
    """
    parts = _REF_SPLIT_RE.split(text or "")
    # parts = [before-first-ref, label1, segment1, label2, segment2, ...]
    boxes: list[dict] = []
    for i in range(0, len(parts), 2):
        label = fallback_label if i == 0 else (parts[i - 1].strip() or fallback_label)
        for b in parse_boxes(parts[i], image_width, image_height):
            b["label"] = label
            boxes.append(b)
    return boxes


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
