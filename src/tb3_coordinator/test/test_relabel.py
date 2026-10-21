#!/usr/bin/env python3
"""
test_relabel.py — unit tests for the landmark relabelling rule.

Pure-function layer only: `should_relabel` plus the `Landmark` evidence
bookkeeping. No ROS graph, no Gazebo — run it directly:

    source /opt/ros/humble/setup.bash
    python3 src/tb3_coordinator/test/test_relabel.py

or under pytest:

    python3 -m pytest src/tb3_coordinator/test/test_relabel.py -q
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))

from tb3_coordinator.semantic_map_memory_node import (  # noqa: E402
    Landmark, should_relabel)

# The shipped thresholds (coordinator.yaml).
MIN_OBS = {"person": 8, "chair": 12, "trash_can": 12}
RATIO = 1.5
DEFAULT_MIN = 3


def relabel(counts, current, ratio=RATIO):
    return should_relabel(counts, current, ratio, MIN_OBS, DEFAULT_MIN)


# ── the case this feature exists for ────────────────────────────────────
def test_wrong_then_right_relabels():
    """0905 run3: trash_can promoted first, chair evidence piles up after."""
    assert relabel({"trash_can": 12, "chair": 86}, "trash_can") == "chair"


def test_replay_0905_run3_timeline():
    """Replay the real ordering: 12 trash_can arrive, then 86 chair.

    Asserts the label is trash_can until the chair count clears BOTH gates
    and is chair by the end — i.e. the correction happens during the run,
    not only in hindsight.
    """
    lm = Landmark(landmark_id="trash_can_1", semantic_class="trash_can",
                  x=0.5, y=1.4, class_counts={})
    for _ in range(12):
        lm.class_counts["trash_can"] = lm.class_counts.get("trash_can", 0) + 1
    assert lm.semantic_class == "trash_can"
    assert lm.observation_count == 12

    switched_at = None
    for i in range(1, 87):
        lm.class_counts["chair"] = i
        new = relabel(lm.class_counts, lm.semantic_class)
        if new is not None:
            switched_at = i
            lm.semantic_class = new
            lm.landmark_id = "chair_0"
            break

    # 1.5 x 12 = 18, and chair's own min_observations is 12, so 18 governs.
    assert switched_at == 18, switched_at
    assert lm.semantic_class == "chair"
    # the current-label count follows the relabel
    lm.class_counts["chair"] = 86
    assert lm.observation_count == 86


# ── the guards ──────────────────────────────────────────────────────────
def test_below_ratio_does_not_relabel():
    """Past its own min_observations, but not 1.5x ahead."""
    assert relabel({"trash_can": 12, "chair": 17}, "trash_can") is None
    assert relabel({"trash_can": 12, "chair": 18}, "trash_can") == "chair"


def test_below_min_obs_does_not_relabel():
    """Way past the ratio, but chair has not earned its own threshold."""
    assert relabel({"trash_can": 5, "chair": 11}, "trash_can") is None
    assert relabel({"trash_can": 5, "chair": 12}, "trash_can") == "chair"


def test_equal_counts_do_not_relabel():
    assert relabel({"trash_can": 30, "chair": 30}, "trash_can") is None
    assert relabel({"trash_can": 12, "chair": 12}, "trash_can") is None


def test_unknown_label_uses_default_min_obs():
    """A label with no configured threshold falls back to min_observations."""
    assert relabel({"chair": 10, "bench": 2}, "chair") is None
    assert relabel({"chair": 2, "bench": 3}, "chair") == "bench"


def test_no_challenger_at_all():
    assert relabel({"chair": 50}, "chair") is None
    assert relabel({}, "chair") is None


# ── hysteresis: flipping back is deliberately harder ────────────────────
def test_reverse_flip_needs_the_ratio_again():
    """After chair wins at 86, trash_can needs 1.5 x 86 = 129 to take it back."""
    counts = {"trash_can": 12, "chair": 86}
    assert relabel(counts, "trash_can") == "chair"

    counts["trash_can"] = 128
    assert relabel(counts, "chair") is None, "128 < 1.5*86 must not flip back"
    counts["trash_can"] = 129
    assert relabel(counts, "chair") == "trash_can"


def test_no_oscillation_at_the_boundary():
    """Straddling the threshold must not produce a flip on every observation."""
    counts = {"trash_can": 12, "chair": 18}
    assert relabel(counts, "trash_can") == "chair"
    # now labelled chair with 18; trash_can at 12 is nowhere near 1.5*18=27
    assert relabel(counts, "chair") is None


# ── id sequencing after a relabel ───────────────────────────────────────
def test_id_sequence_after_relabel():
    """New id comes from the NEW class's sequence; the old id is retired."""
    landmarks = {}
    next_seq = {}

    def promote(label, x, y, n):
        seq = next_seq.get(label, 0)
        next_seq[label] = seq + 1
        lid = "%s_%d" % (label, seq)
        landmarks[lid] = Landmark(landmark_id=lid, semantic_class=label,
                                  x=x, y=y, class_counts={label: n})
        return landmarks[lid]

    chair_a = promote("chair", 3.0, 3.0, 40)       # a real chair elsewhere
    ghost = promote("trash_can", 0.5, 1.4, 12)      # the mislabelled one
    assert chair_a.landmark_id == "chair_0"
    assert ghost.landmark_id == "trash_can_0"

    ghost.class_counts["chair"] = 86
    new_label = relabel(ghost.class_counts, ghost.semantic_class)
    assert new_label == "chair"

    old_id = ghost.landmark_id
    seq = next_seq.get(new_label, 0)
    next_seq[new_label] = seq + 1
    landmarks.pop(old_id)
    ghost.landmark_id = "%s_%d" % (new_label, seq)
    ghost.semantic_class = new_label
    landmarks[ghost.landmark_id] = ghost

    assert ghost.landmark_id == "chair_1", ghost.landmark_id
    assert "trash_can_0" not in landmarks, "old id must be retired"
    assert set(landmarks) == {"chair_0", "chair_1"}
    # the retired trash_can sequence is not reused either
    assert next_seq["trash_can"] == 1


# ── same-class targets must never challenge each other ──────────────────
def test_same_class_never_challenges_itself():
    """Five-person world: person evidence on a person landmark is not a
    challenger, however large it gets."""
    assert relabel({"person": 500}, "person") is None
    lm = Landmark(landmark_id="person_0", semantic_class="person",
                  x=1.0, y=1.0, class_counts={"person": 500})
    assert relabel(lm.class_counts, lm.semantic_class) is None
    assert lm.observation_count == 500


# ── Landmark bookkeeping ────────────────────────────────────────────────
def test_observation_count_tracks_current_label():
    lm = Landmark(landmark_id="trash_can_1", semantic_class="trash_can",
                  x=0.0, y=0.0, class_counts={"trash_can": 12, "chair": 86})
    assert lm.observation_count == 12
    lm.semantic_class = "chair"
    assert lm.observation_count == 86


def test_counts_summary_orders_by_strength():
    lm = Landmark(landmark_id="x_0", semantic_class="chair", x=0.0, y=0.0,
                  class_counts={"trash_can": 12, "chair": 86, "person": 12})
    assert lm.counts_summary() == "chair:86,person:12,trash_can:12"
    assert Landmark("y_0", "chair", 0.0, 0.0).counts_summary() == "-"


def test_ratio_is_configurable():
    counts = {"trash_can": 12, "chair": 15}
    assert relabel(counts, "trash_can", ratio=1.5) is None
    assert relabel(counts, "trash_can", ratio=1.2) == "chair"


def _main():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print("  PASS  %s" % name)
        except AssertionError as exc:
            failed += 1
            print("  FAIL  %s: %s" % (name, exc))
    print("\n%d/%d passed" % (len(tests) - failed, len(tests)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_main())
