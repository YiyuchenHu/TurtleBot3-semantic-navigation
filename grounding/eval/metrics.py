"""Scoring primitives for the grounding eval harness.

Pure python, stdlib only — cv2/numpy-FREE by repo convention so pytest
runs on dev machines without the container.

Metric definitions
------------------
ground mode (one image, one query, one ground-truth box):
    hit  <=>  (a) at least one predicted box has IoU >= threshold (0.5)
              with gt_bbox_xyxy, AND
              (b) if distractor boxes are given: the *top-scoring*
              predicted box overlaps the ground truth strictly more
              than it overlaps every distractor. This is the spatial
              distractor-rejection check — the model's primary answer
              must point at the right instance, not merely include it
              somewhere in its box list.

rank mode (several candidates, one query):
    Each candidate scores max(IoU(pred, ref_bbox) * pred.score) over the
    predicted boxes for its frame; the case passes if the argmax
    candidate is the expected winner (top-1 accuracy). This deliberately
    reimplements the small IoU x score ranking from
    src/tb3_grounding/tb3_grounding/resolver_core.py (score_candidate /
    rank_candidates) rather than importing across source trees.
"""

from __future__ import annotations

from typing import Optional

IOU_THRESHOLD = 0.5


def iou(a_xyxy: list, b_xyxy: list) -> float:
    """Intersection-over-union of two [x1, y1, x2, y2] boxes.

    Degenerate (zero-area) boxes and non-overlapping boxes score 0.0.
    """
    ax1, ay1, ax2, ay2 = a_xyxy
    bx1, by1, bx2, by2 = b_xyxy
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def top_scoring_box(boxes: list[dict]) -> Optional[dict]:
    """The predicted box with the highest score (first wins ties)."""
    best = None
    best_score = float("-inf")
    for box in boxes:
        s = float(box.get("score", 1.0))
        if s > best_score:
            best_score = s
            best = box
    return best


def ground_hit(
    pred_boxes: list[dict],
    gt_bbox_xyxy: list,
    distractor_bboxes: Optional[list[list]] = None,
    iou_threshold: float = IOU_THRESHOLD,
) -> dict:
    """Score one ground-mode case. See module docstring for the rule.

    Returns a detail dict:
        hit                  bool — overall verdict
        matched              bool — condition (a): some box IoU >= thr vs gt
        distractor_rejected  bool | None — condition (b); None when no
                             distractors were given
        best_gt_iou          float — best IoU of any predicted box vs gt
        top_box_gt_iou       float | None — top-scoring box's IoU vs gt
        n_boxes              int
    """
    best_gt_iou = 0.0
    for box in pred_boxes:
        j = iou(box["bbox_xyxy"], gt_bbox_xyxy)
        if j > best_gt_iou:
            best_gt_iou = j
    matched = best_gt_iou >= iou_threshold

    distractor_rejected: Optional[bool] = None
    top_box_gt_iou: Optional[float] = None
    if distractor_bboxes:
        top = top_scoring_box(pred_boxes)
        if top is None:
            distractor_rejected = False
        else:
            top_box_gt_iou = iou(top["bbox_xyxy"], gt_bbox_xyxy)
            distractor_rejected = all(
                top_box_gt_iou > iou(top["bbox_xyxy"], d)
                for d in distractor_bboxes
            )

    hit = matched and (distractor_rejected is not False)
    return {
        "hit": hit,
        "matched": matched,
        "distractor_rejected": distractor_rejected,
        "best_gt_iou": round(best_gt_iou, 4),
        "top_box_gt_iou": (
            None if top_box_gt_iou is None else round(top_box_gt_iou, 4)
        ),
        "n_boxes": len(pred_boxes),
    }


def score_candidate(boxes: list[dict], ref_bbox_xyxy: list) -> float:
    """max(IoU x box score) of the predicted boxes vs the candidate's
    reference bbox. Mirrors resolver_core.score_candidate."""
    best = 0.0
    for box in boxes:
        s = iou(box["bbox_xyxy"], ref_bbox_xyxy) * float(box.get("score", 1.0))
        if s > best:
            best = s
    return best


def rank_top1(candidate_scores: dict, expected_winner: str) -> dict:
    """Top-1 verdict for one rank-mode case.

    candidate_scores: {candidate_id: score} (from score_candidate).
    The winner is the highest-scoring candidate; a case with all-zero
    scores has no winner (the model grounded nothing near any ref box).
    """
    winner: Optional[str] = None
    best = 0.0
    for cid, s in candidate_scores.items():
        if s > best:
            best = s
            winner = cid
    return {
        "correct": winner == expected_winner,
        "winner": winner,
        "expected": expected_winner,
        "scores": {cid: round(s, 4) for cid, s in candidate_scores.items()},
    }
