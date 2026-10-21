#!/usr/bin/env python3
"""
test_promotion_guard.py — unit tests for the distinct-detection promotion
guard (`min_distinct_observations`).

Pure-function layer only: `promotion_ready` plus the `Candidate` key
bookkeeping, driven by the two sequences the guard was designed on. No ROS
graph — run it directly:

    source /opt/ros/humble/setup.bash
    python3 src/tb3_coordinator/test/test_promotion_guard.py

or under pytest:

    python3 -m pytest src/tb3_coordinator/test/test_promotion_guard.py -q
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))

from tb3_coordinator.semantic_map_memory_node import (  # noqa: E402
    Candidate, promotion_ready)

MIN_OBS = 8        # coordinator.yaml min_observations (generic classes)
SEARCH_MIN = 3     # search_min_observations
MIN_DISTINCT = 3   # min_distinct_observations


def feed(cand, obs_id, stamp_s, min_obs=MIN_OBS):
    """One memory message for `cand`; returns True when it would promote."""
    cand.obs_count += 1
    cand.obs_keys.add((obs_id, int(round(stamp_s * 1e9))))
    return promotion_ready(cand.obs_count, cand.distinct_count,
                           min_obs, MIN_DISTINCT)


def new_cand(label, obs_id, stamp_s):
    return Candidate(semantic_class=label, x=0.0, y=0.0, obs_count=1,
                     obs_keys={(obs_id, int(round(stamp_s * 1e9)))})


# ── the case this guard exists for ───────────────────────────────────────
def test_r07_smoke_ghost_is_held():
    """Recorded test run, +214 s: two mislabelled
    "trash can" frames on the green-shirt person, stamps 1789544917.52 and
    1789544919.99, became memory objects trash can_3 / trash can_4, each
    republished at 1 Hz onto the same map point -> the candidate went
    n=3 -> 5 -> 7 -> 8 in 4 s and was promoted from 2 real detections.
    """
    c = new_cand("trash can", "trash can_3", 1789544917.52)
    promoted = False
    # 1 Hz republishes: both objects live for the 5 s stale window
    for _ in range(3):
        promoted |= feed(c, "trash can_3", 1789544917.52)
    for _ in range(4):
        promoted |= feed(c, "trash can_4", 1789544919.99)
    assert c.obs_count == 8, "count reaches the old threshold"
    assert c.distinct_count == 2
    assert promoted is False, "8 stacked republishes of 2 detections must not promote"
    # more republishes of the same two detections never help
    for _ in range(10):
        assert feed(c, "trash can_4", 1789544919.99) is False
    # a third distinct detection does
    assert feed(c, "trash can_4", 1789544923.10) is True


# ── real objects must be unaffected ──────────────────────────────────────
def test_bag_ab_barrier_still_promotes():
    """Recorded bag replay (stamped instance): the barrier candidate
    at (-2.18,-1.45) reached n=8 on memory object concrete barrier_0 with
    stamps 41.53, 45.03, 45.90 s (rows json) -> 3 distinct -> promotes at
    the same message as before the guard.
    """
    seq = [41.53, 41.53, 41.53, 45.03, 45.03, 45.90, 45.90, 45.90]
    c = new_cand("concrete barrier", "concrete barrier_0", seq[0])
    when = None
    for i, st in enumerate(seq[1:], start=2):
        if feed(c, "concrete barrier_0", st):
            when = i
            break
    assert when == 8, "promotes exactly when n reaches 8, as before"
    assert c.distinct_count == 3


def test_stationary_robot_many_frames_one_object():
    """Robot parked in front of the shelf: one memory object, a new
    detection every ~1.3 s (detector req_rate 0.76/s) -> distinct stamps
    accumulate faster than the count threshold, guard never binds."""
    c = new_cand("bookshelf", "bookshelf_0", 100.0)
    st = 100.0
    for i in range(2, 9):
        st += 1.3
        r = feed(c, "bookshelf_0", st)
        assert r is (i >= MIN_OBS)


# ── search classes: same 3, on top of search_min_observations 3 ─────────
def test_search_class_needs_three_frames_not_three_republishes():
    """Green-shirt search: one frame republished 3x used to promote at
    n=3; now it needs 3 distinct frames (offline probe: 6/60 frames carry the
    box, so 3 frames arrive within one sweep)."""
    label = "person wearing a green shirt"
    c = new_cand(label, label + "_0", 10.0)
    assert feed(c, label + "_0", 10.0, SEARCH_MIN) is False
    assert feed(c, label + "_0", 10.0, SEARCH_MIN) is False   # n=3, k=1
    assert feed(c, label + "_0", 11.4, SEARCH_MIN) is False   # n=4, k=2
    assert feed(c, label + "_0", 12.7, SEARCH_MIN) is True    # n=5, k=3


# ── both halves of the rule ─────────────────────────────────────────────
def test_pure_rule():
    assert promotion_ready(8, 3, 8, 3)
    assert promotion_ready(20, 7, 8, 3)
    assert not promotion_ready(8, 2, 8, 3), "count ok, distinct short"
    assert not promotion_ready(7, 7, 8, 3), "distinct ok, count short"
    assert promotion_ready(8, 1, 8, 1), "min_distinct 1 == old behaviour"
    assert promotion_ready(3, 3, 3, 3)


def test_unstamped_keys_are_unique():
    """Legacy input without stamps: the node keys each observation
    uniquely (negative sequence), so the guard cannot starve anything."""
    c = Candidate(semantic_class="person", x=0, y=0, obs_count=1,
                  obs_keys={("person_0", -1)})
    for i in range(2, 9):
        c.obs_count += 1
        c.obs_keys.add(("person_0", -i))
    assert promotion_ready(c.obs_count, c.distinct_count, 8, 3)


if __name__ == "__main__":
    import inspect
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and inspect.isfunction(fn):
            try:
                fn()
                print("PASS", name)
            except AssertionError as e:
                failures += 1
                print("FAIL", name, "-", e)
    sys.exit(1 if failures else 0)
