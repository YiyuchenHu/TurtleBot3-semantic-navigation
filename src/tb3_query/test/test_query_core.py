#!/usr/bin/env python3
"""
test_query_core.py — unit tests for the pure query layer (R07):
expression canonicalisation and attribute-based target selection.

No ROS graph — run directly or under pytest:

    python3 src/tb3_query/test/test_query_core.py
    python3 -m pytest src/tb3_query/test/test_query_core.py -q
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))

from tb3_query.query_core import (  # noqa: E402
    MemoryObject, canonicalize_expression, extract_expression,
    select_by_attribute, select_target)


# ── wording the detector grounds ────────────────────────────────────────
def test_canonicalize_paraphrases():
    """2026-09-17: 'person ware green t shirt' produced 0 boxes in 138 s."""
    assert canonicalize_expression("person ware green t shirt") == "person wearing a green shirt"
    assert canonicalize_expression("person ware green T-shirt") == "person wearing a green shirt"
    assert canonicalize_expression("person in a red top") == "person wearing a red shirt"
    assert canonicalize_expression("person with blue tshirt") == "person wearing a blue shirt"
    assert canonicalize_expression("the person who is wearing a white shirt") == "person wearing a white shirt"


def test_canonicalize_leaves_others_alone():
    assert canonicalize_expression("person wearing a green shirt") == "person wearing a green shirt"
    assert canonicalize_expression("blue chair") == "blue chair"
    assert canonicalize_expression("bicycle") == "bicycle"
    assert canonicalize_expression("person") == "person"


def test_end_to_end_from_command_text():
    expr = extract_expression("go to the person ware green T-shirt")
    assert expr == "person ware green t shirt"   # extract_expression normalises case/punctuation
    assert canonicalize_expression(expr) == "person wearing a green shirt"


# ── attribute resolution ────────────────────────────────────────────────
GREEN = "person wearing a green shirt"


def _objs():
    return [
        MemoryObject("person_0", "person", -1.1, 1.8, 26.0, {GREEN: 1.0}),   # white person, 1 stray green box
        MemoryObject("person_2", "person", 0.1, -0.06, 44.0, {GREEN: 5.0}),  # the green person
        MemoryObject("bookshelf_0", "bookshelf", 0.0, 2.5, 29.0),
    ]


def test_attribute_picks_the_strongest_evidence():
    best = select_by_attribute(_objs(), GREEN, min_count=2)
    assert best is not None and best.object_id == "person_2"


def test_attribute_min_count_filters_strays():
    assert select_by_attribute(_objs(), GREEN, min_count=6) is None
    objs = [MemoryObject("person_0", "person", 0, 0, 10.0, {GREEN: 1.0})]
    assert select_by_attribute(objs, GREEN, min_count=2) is None


def test_select_target_uses_attribute_when_no_class_match():
    r = select_target(_objs(), GREEN, GREEN, "go to the person wearing a green shirt")
    assert r.success and r.object_id == "person_2"
    assert "by attribute" in r.status_message
    assert r.confidence == 5.0


def test_select_target_prefers_exact_class_landmark():
    objs = _objs() + [MemoryObject(GREEN + "_0", GREEN, 0.1, -0.06, 14.0)]
    r = select_target(objs, GREEN, GREEN, "find the person wearing a green shirt")
    assert r.success and r.object_id == GREEN + "_0"


def test_indexed_query_never_uses_attributes():
    r = select_target(_objs(), GREEN, GREEN, "go to person wearing a green shirt 1", desired_index=1)
    assert not r.success


def test_plain_class_query_unaffected():
    r = select_target(_objs(), "person", "person", "go to the person")
    assert r.success and r.object_id == "person_2"   # nearest


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
