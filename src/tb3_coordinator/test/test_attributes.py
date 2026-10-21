#!/usr/bin/env python3
"""
test_attributes.py — attribute-on-identity bookkeeping.

Pure layer: `Landmark.attr_keys` / `attr_count` / `top_attribute`, the
marker label, and the strict token-subset rule `_find_family_landmark`
uses. No ROS graph:

    python3 src/tb3_coordinator/test/test_attributes.py
    python3 -m pytest src/tb3_coordinator/test/test_attributes.py -q
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))

from tb3_coordinator.semantic_map_memory_node import (  # noqa: E402
    Landmark, _marker_label, labels_same_family)

GREEN = "person wearing a green shirt"


def _person():
    return Landmark(landmark_id="person_2", semantic_class="person",
                    x=0.1, y=-0.06, class_counts={"person": 44})


def test_attr_count_is_distinct_detections_not_republishes():
    """1 Hz republishes of one detection share (id, stamp); 5 messages from
    2 detections must count 2 (same rule as the promotion guard)."""
    lm = _person()
    for key in [("g_0", 100), ("g_0", 100), ("g_0", 100), ("g_1", 104), ("g_1", 104)]:
        lm.attr_keys.setdefault(GREEN, set()).add(key)
    assert lm.attr_count(GREEN) == 2
    assert lm.attr_count("person wearing a red shirt") == 0
    assert lm.top_attribute() == (GREEN, 2)


def test_strongest_attribute_wins():
    lm = _person()
    lm.attr_keys[GREEN] = {("a", 1), ("a", 2), ("a", 3)}
    lm.attr_keys["person wearing a blue shirt"] = {("b", 1)}
    assert lm.top_attribute() == (GREEN, 3)
    assert lm.attrs_summary().startswith(GREEN + ":3")


def test_marker_label_shows_attribute_without_spaces():
    lm = _person()
    lm.attr_keys[GREEN] = {("a", 1), ("a", 2)}
    label = _marker_label(lm)
    assert label == "person_2\nn44\ngreen_shirt:2"
    assert " " not in label      # rviz2 renders a space as a wide gap


def test_marker_label_without_attributes_unchanged():
    assert _marker_label(_person()) == "person_2\nn44"


def test_family_rule_is_strict_subset():
    """`person` is the base of the expression; the expression is not the
    base of `person`, and unrelated classes are not family at all."""
    base, expr = set("person".split()), set(GREEN.split())
    assert base < expr
    assert not (expr < base)
    assert labels_same_family("person", GREEN)
    assert not labels_same_family("trash can", GREEN)
    assert not (set("trash can".split()) < expr)


if __name__ == "__main__":
    import inspect
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and inspect.isfunction(fn):
            try:
                fn(); print("PASS", name)
            except AssertionError as e:
                failures += 1; print("FAIL", name, "-", e)
    sys.exit(1 if failures else 0)
