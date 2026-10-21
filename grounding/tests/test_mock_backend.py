import pathlib
import sys

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from backends.mock import MockColorBackend  # noqa: E402


def _encode(bgr) -> bytes:
    ok, buf = cv2.imencode(".png", bgr)
    assert ok
    return buf.tobytes()


def _two_color_scene() -> bytes:
    """Grey image, red block left, blue block right."""
    img = np.full((200, 400, 3), 128, dtype=np.uint8)
    img[50:150, 40:160] = (0, 0, 220)    # red (BGR)
    img[50:150, 240:360] = (220, 60, 0)  # blue
    return _encode(img)


def test_warm_query_finds_red_block():
    backend = MockColorBackend()
    backend.load()
    boxes = backend.ground(_two_color_scene(), "sofa with warm color")
    assert boxes, "expected at least one warm region"
    x1, y1, x2, y2 = boxes[0]["bbox_xyxy"]
    assert x2 <= 200
    assert 30 <= x1 <= 50 and 140 <= x2 <= 170


def test_cool_query_finds_blue_block():
    backend = MockColorBackend()
    boxes = backend.ground(_two_color_scene(), "the sofa with cool color")
    assert boxes
    x1, _, x2, _ = boxes[0]["bbox_xyxy"]
    assert x1 >= 200, "cool query should land on the right (blue) block"


def test_named_color():
    backend = MockColorBackend()
    boxes = backend.ground(_two_color_scene(), "red couch")
    assert boxes
    assert boxes[0]["bbox_xyxy"][2] <= 200


def test_no_color_term_returns_empty():
    backend = MockColorBackend()
    assert backend.ground(_two_color_scene(), "the fluffy sofa") == []


def test_detect_labels_boxes_with_their_category():
    backend = MockColorBackend()
    backend.load()
    dets = backend.detect(_two_color_scene(), ["red box", "blue box", "person"])
    labels = {d["label"] for d in dets}
    assert "red box" in labels and "blue box" in labels
    assert "person" not in labels  # no color term -> honest negative
    for d in dets:
        if d["label"] == "red box":
            assert d["bbox_xyxy"][2] <= 200
        if d["label"] == "blue box":
            assert d["bbox_xyxy"][0] >= 200
