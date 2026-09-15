import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from eval import metrics  # noqa: E402


def _box(x1, y1, x2, y2, score=1.0):
    return {"bbox_xyxy": [x1, y1, x2, y2], "score": score, "label": ""}


# ---------------------------------------------------------------- iou

def test_iou_identical_boxes():
    assert metrics.iou([0, 0, 10, 10], [0, 0, 10, 10]) == pytest.approx(1.0)


def test_iou_disjoint_boxes():
    assert metrics.iou([0, 0, 10, 10], [20, 20, 30, 30]) == 0.0


def test_iou_touching_edges_is_zero():
    assert metrics.iou([0, 0, 10, 10], [10, 0, 20, 10]) == 0.0


def test_iou_partial_overlap_value():
    # inter = 1x2 = 2, union = 4 + 4 - 2 = 6
    assert metrics.iou([0, 0, 2, 2], [1, 0, 3, 2]) == pytest.approx(1 / 3)


def test_iou_contained_box():
    # inter = 1, union = 16
    assert metrics.iou([0, 0, 4, 4], [1, 1, 2, 2]) == pytest.approx(1 / 16)


def test_iou_degenerate_zero_area():
    assert metrics.iou([5, 5, 5, 5], [0, 0, 10, 10]) == 0.0
    assert metrics.iou([5, 5, 5, 5], [5, 5, 5, 5]) == 0.0


# --------------------------------------------------------- ground_hit

GT = [10, 10, 50, 50]
DISTRACTOR = [100, 100, 140, 140]


def test_ground_hit_no_boxes_misses():
    r = metrics.ground_hit([], GT)
    assert not r["hit"]
    assert not r["matched"]
    assert r["n_boxes"] == 0


def test_ground_hit_exact_box_hits():
    r = metrics.ground_hit([_box(10, 10, 50, 50)], GT)
    assert r["hit"]
    assert r["best_gt_iou"] == pytest.approx(1.0)
    assert r["distractor_rejected"] is None  # no distractors given


def test_ground_hit_any_box_counts_not_just_first():
    boxes = [_box(200, 200, 220, 220, score=0.9), _box(11, 11, 49, 49, 0.3)]
    assert metrics.ground_hit(boxes, GT)["hit"]


def test_ground_hit_below_threshold_misses():
    # IoU of [10,10,30,30] vs GT = 400/1600 = 0.25 < 0.5
    r = metrics.ground_hit([_box(10, 10, 30, 30)], GT)
    assert not r["hit"]
    assert r["best_gt_iou"] == pytest.approx(0.25)


def test_ground_hit_distractor_rejection_fails_when_top_box_on_distractor():
    # A gt-aligned box exists, but the top-SCORING box sits on the
    # distractor -> the model's primary answer is wrong.
    boxes = [
        _box(*DISTRACTOR, score=0.9),
        _box(10, 10, 50, 50, score=0.4),
    ]
    r = metrics.ground_hit(boxes, GT, [DISTRACTOR])
    assert r["matched"]
    assert r["distractor_rejected"] is False
    assert not r["hit"]


def test_ground_hit_distractor_rejection_passes_when_top_box_on_gt():
    boxes = [
        _box(10, 10, 50, 50, score=0.9),
        _box(*DISTRACTOR, score=0.4),
    ]
    r = metrics.ground_hit(boxes, GT, [DISTRACTOR])
    assert r["hit"]
    assert r["distractor_rejected"] is True


def test_ground_hit_distractors_with_no_predictions():
    r = metrics.ground_hit([], GT, [DISTRACTOR])
    assert not r["hit"]
    assert r["distractor_rejected"] is False


# ---------------------------------------------------- rank / candidate

def test_score_candidate_is_max_iou_times_score():
    ref = [0, 0, 10, 10]
    boxes = [
        _box(0, 0, 10, 10, score=0.5),   # iou 1.0 -> 0.5
        _box(0, 0, 5, 10, score=1.0),    # iou 0.5 -> 0.5
        _box(50, 50, 60, 60, score=1.0),  # iou 0 -> 0
    ]
    assert metrics.score_candidate(boxes, ref) == pytest.approx(0.5)
    assert metrics.score_candidate([], ref) == 0.0


def test_rank_top1_correct_winner():
    r = metrics.rank_top1({"a": 0.8, "b": 0.2}, "a")
    assert r["correct"]
    assert r["winner"] == "a"


def test_rank_top1_wrong_winner():
    r = metrics.rank_top1({"a": 0.1, "b": 0.7}, "a")
    assert not r["correct"]
    assert r["winner"] == "b"


def test_rank_top1_all_zero_scores_has_no_winner():
    r = metrics.rank_top1({"a": 0.0, "b": 0.0}, "a")
    assert not r["correct"]
    assert r["winner"] is None
