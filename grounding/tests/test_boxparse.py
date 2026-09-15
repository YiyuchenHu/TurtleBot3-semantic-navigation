import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from boxparse import parse_boxes, parse_labeled_boxes  # noqa: E402


def test_single_box_scales_to_pixels():
    text = "<box><100><200><500><800></box>"
    boxes = parse_boxes(text, image_width=1000, image_height=500)
    assert len(boxes) == 1
    assert boxes[0]["bbox_xyxy"] == [100.0, 100.0, 500.0, 400.0]
    assert boxes[0]["score"] == 1.0


def test_multiple_boxes_preserve_order():
    text = (
        "here <box><0><0><100><100></box> and "
        "<box><500><500><1000><1000></box> done"
    )
    boxes = parse_boxes(text, 200, 200)
    assert len(boxes) == 2
    assert boxes[0]["bbox_xyxy"] == [0.0, 0.0, 20.0, 20.0]
    assert boxes[1]["bbox_xyxy"] == [100.0, 100.0, 200.0, 200.0]


def test_out_of_range_coords_are_clamped():
    text = "<box><900><900><1200><1200></box>"
    boxes = parse_boxes(text, 100, 100)
    assert boxes[0]["bbox_xyxy"] == [90.0, 90.0, 100.0, 100.0]


def test_degenerate_box_dropped():
    text = "<box><500><500><500><900></box>"  # zero width
    assert parse_boxes(text, 640, 480) == []


def test_garbage_and_empty_text():
    assert parse_boxes("no boxes here", 640, 480) == []
    assert parse_boxes("", 640, 480) == []
    assert parse_boxes("<box><1><2><3></box>", 640, 480) == []  # 3 coords


# ── parse_labeled_boxes (detect-mode grouped output) ─────────────────────

def test_labeled_groups_from_detect_output():
    # Shape taken from a real R01 §2.3 detect() answer: per-category
    # groups, one category cross-listed, one honest None.
    text = (
        "<ref>armchair</ref><box><0><0><270><623></box>"
        "<box><633><131><991><569></box>"
        "<ref>trash can</ref><box><250><83><348><527></box>"
        "<ref>person</ref><box>None</box><|im_end|>"
    )
    boxes = parse_labeled_boxes(text, 1000, 1000)
    assert [b["label"] for b in boxes] == ["armchair", "armchair", "trash can"]
    assert boxes[0]["bbox_xyxy"] == [0.0, 0.0, 270.0, 623.0]
    assert boxes[2]["bbox_xyxy"] == [250.0, 83.0, 348.0, 527.0]


def test_labeled_boxes_before_any_ref_get_fallback():
    text = "<box><100><100><300><300></box><ref>cat</ref><box><400><400><600><600></box>"
    boxes = parse_labeled_boxes(text, 1000, 1000, fallback_label="the query")
    assert [b["label"] for b in boxes] == ["the query", "cat"]


def test_labeled_empty_ref_uses_fallback():
    text = "<ref>  </ref><box><100><100><300><300></box>"
    boxes = parse_labeled_boxes(text, 1000, 1000, fallback_label="q")
    assert [b["label"] for b in boxes] == ["q"]


def test_labeled_none_and_garbage():
    assert parse_labeled_boxes("<ref>person</ref><box>None</box>", 640, 480) == []
    assert parse_labeled_boxes("", 640, 480) == []
    assert parse_labeled_boxes("no refs no boxes", 640, 480) == []
