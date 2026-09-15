#!/usr/bin/env python3
"""
query_core.py — Pure-Python logic for Stage-4 semantic query.

Three responsibilities:
    1. parse_command()    — deterministic text → canonical semantic_name
    2. load_target_mapping() — YAML → semantic_name ↔ detector_label dicts
    3. select_target()    — pick the nearest matching object from memory

No ROS dependency.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


# ── Data types ────────────────────────────────────────────────────────────

@dataclass
class MemoryObject:
    """Lightweight mirror of one Detection3D from the memory topic."""
    object_id: str
    detector_label: str
    x: float
    y: float
    confidence: float


@dataclass
class QueryResult:
    """Outcome of one semantic query."""
    success: bool
    query_text: str
    semantic_name: str = ""
    detector_label: str = ""
    object_id: str = ""
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    status_message: str = ""


@dataclass
class ParsedCommand:
    """Outcome of parsing a free-form text command.

    Examples
    --------
    "go to person"        -> ParsedCommand("person", desired_index=None)
    "go to person 3"      -> ParsedCommand("person", desired_index=3)
    "go to person_3"      -> ParsedCommand("person", desired_index=3)
    "go to person3"       -> ParsedCommand("person", desired_index=3)
    "go to person no. 5"  -> ParsedCommand("person", desired_index=5)
    "find the table"      -> ParsedCommand("table",  desired_index=None)
    """
    semantic_name: str
    desired_index: Optional[int] = None


# ── Mapping loader ────────────────────────────────────────────────────────

def load_target_mapping(yaml_path: str | Path) -> tuple[dict[str, str], dict[str, str]]:
    """Load semantic_targets.yaml and return two lookup dicts.

    Returns:
        (sem2det, det2sem)
        sem2det:  semantic_name  → detector_label
        det2sem:  detector_label → semantic_name
    """
    path = Path(yaml_path)
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    targets = data.get("semantic_targets", [])
    sem2det: dict[str, str] = {}
    det2sem: dict[str, str] = {}
    for entry in targets:
        if not entry.get("enabled", True):
            continue
        sn = entry["semantic_name"]
        dl = entry["detector_label"]
        sem2det[sn] = dl
        det2sem[dl] = sn

    return sem2det, det2sem


# ── Command parser ────────────────────────────────────────────────────────

_FILLER = frozenset({
    "go", "to", "the", "a", "an", "please", "navigate", "approach",
    "find", "me", "take", "bring", "get", "can", "you", "i",
    "want", "need", "would", "like", "could", "help", "with",
})

# Words that may appear *between* a target token and an integer (e.g.
# "person number 3", "person no 5"). Skipped while scanning for the index.
_INDEX_FILLER = frozenset({"number", "no", "num"})

# ── Open-vocabulary expression extraction (Phase 2c search link) ──────────
#
# Words that may LEAD a navigation command before the target expression
# starts. Deliberately a separate set from _FILLER: these are only
# stripped from the FRONT of the command, so interior words of the
# expression survive ("go to the box on the armchair" keeps its interior
# "the"/"on"). The articles are in this set on purpose — stripping the
# LEADING article is the de-articling rule the R02b experiment-d retrial
# measured (leading "the" made /detect expression output unstable;
# re-verified live on person-world frames in R02c §2.1 before this
# shipped).
_ARTICLES = frozenset({"the", "a", "an"})
_COMMAND_PREFIX = frozenset({
    "go", "to", "please", "navigate", "approach", "find", "me", "take",
    "bring", "get", "can", "you", "i", "want", "need", "would", "like",
    "could", "help", "with", "drive", "move", "head", "locate", "search",
    "for", "reach", "visit", "us", "look",
}) | _ARTICLES

# Politeness tail that may FOLLOW the expression ("find the bicycle
# please"). Stripped from the back only.
_COMMAND_SUFFIX = frozenset({"please", "now", "thanks", "thank", "you"})


def extract_expression(text: str) -> Optional[str]:
    """Extract the open-vocabulary target expression from a command.

    Used when `parse_command` found no registry target: the remainder of
    the command becomes the expression injected verbatim into the detect
    vocabulary (and the detector_label its landmark will carry).

    Returns None unless the command LOOKS like a navigation command —
    at least one `_COMMAND_PREFIX` token must have been stripped from the
    front. This keeps arbitrary chatter ("hello robot") from starting an
    exploration run, while "go to the bicycle" / "find bicycle" work.

        "go to the blue chair"      -> "blue chair"
        "find the bicycle please"   -> "bicycle"
        "go to the box on the sofa" -> "box on the sofa"   (interior words kept)
        "hello robot"               -> None                (no command prefix)
    """
    tokens = _normalize(text).split()
    i = 0
    while i < len(tokens) and tokens[i] in _COMMAND_PREFIX:
        i += 1
    if i == 0:
        return None
    j = len(tokens)
    while j > i and tokens[j - 1] in _COMMAND_SUFFIX:
        j -= 1
    expression = " ".join(tokens[i:j])
    return expression or None


def attribute_expression(
    text: str,
    semantic_name: str,
    detector_label: str,
) -> Optional[str]:
    """Detect an attribute-qualified registry-class command (R04).

    `parse_command` matches on single tokens, so "find the person wearing
    a green shirt" resolves to the registry class `person` — and a search
    would inject plain "person" and stop at ANY person. If the command
    carries meaningful tokens beyond the class reference itself, the whole
    expression must be treated as open-vocabulary instead.

    Returns the full expression ("person wearing a green shirt") when the
    command has attribute tokens, else None (pure class reference —
    registry path stays in charge).

        "find the person wearing a green shirt" -> "person wearing a green shirt"
        "go to the person"                      -> None
        "go to person 3"                        -> None   (index, not attribute)
        "go to the traffic cone"                -> None   (alias tokens)
    """
    expr = extract_expression(text)
    if expr is None:
        return None
    class_words = set(_normalize(semantic_name).split())
    class_words |= set(_normalize(detector_label).split())
    for phrase, canonical in _PHRASE_ALIASES.items():
        if canonical == semantic_name:
            class_words |= set(_normalize(phrase).split())
    residual = [t for t in _normalize(expr).split()
                if t not in class_words
                and t not in _INDEX_FILLER
                and not t.isdigit()]
    return expr if residual else None

_PHRASE_ALIASES: dict[str, str] = {
    "bench": "table",
    "stop sign": "stop_sign",
    # LocateAnything registry (semantic_targets_la.yaml). Each alias only
    # fires when its canonical name is in the loaded registry, so the
    # YOLO path (semantic_targets.yaml, which has none of these) is
    # unaffected.
    "trash can": "trash_can",
    "trashcan": "trash_can",
    "garbage can": "trash_can",
    "traffic cone": "cone",
    "construction cone": "cone",
    "shelf": "bookshelf",
    "traffic barrier": "traffic_barrier",
    "jersey barrier": "traffic_barrier",
    "barrier": "traffic_barrier",
}


def _normalize(text: str) -> str:
    r"""Lower-case, strip punctuation, and split glued forms.

    Three normalizations matter for the "person N" feature:

    1. Replace any non-alphanumeric, non-underscore, non-space char with
       a space. This already happened in the previous version.
    2. Convert every underscore to a space so "person_3" is split into
       two tokens. (Underscore is ``\w``, so it survives step 1 by
       itself.)
    3. Insert a space between any letter-followed-by-digit boundary so
       "person3" becomes "person 3". Without this, "person3" would land
       as a single non-known-target token and the whole command would
       be rejected.

    The result is then collapsed whitespace, suitable for ``.split()``.
    """
    norm = re.sub(r"[^\w\s]", " ", (text or "").lower())
    norm = norm.replace("_", " ")
    norm = re.sub(r"([a-z])(\d)", r"\1 \2", norm)
    norm = re.sub(r"\s+", " ", norm).strip()
    return norm


def _scan_index(tokens: list[str]) -> Optional[int]:
    """Look in the next few tokens for an integer, skipping known fillers.

    Used after the target token is located. Stops at the first
    non-filler, non-numeric token so we don't pick up indices that
    appear mid-sentence after another concept (e.g. "go to person and
    table 4" should not return 4 for `person`).
    """
    for tok in tokens[:4]:
        if tok in _FILLER or tok in _INDEX_FILLER:
            continue
        if tok.isdigit():
            return int(tok)
        # Hit a non-numeric, non-filler token — index list ends here.
        return None
    return None


def parse_command(
    text: str,
    known_targets: set[str],
) -> Optional[ParsedCommand]:
    """Parse a text command into a `ParsedCommand`, or `None` if no
    target is recognized.

    Supports:
      * multi-word phrases via `_PHRASE_ALIASES` ("stop sign", "bench"),
      * trailing integer index ("person 3" / "person_3" / "person3" /
        "person number 3" / "person no 5"),
      * filler words listed in `_FILLER` and `_INDEX_FILLER`.
    """
    norm = _normalize(text)
    if not norm:
        return None

    # 1. Multi-word phrase aliases first ("stop sign", "bench" → ...).
    #    Phrases are normalized identically (lower-case, no punctuation).
    for phrase, canonical in _PHRASE_ALIASES.items():
        if canonical not in known_targets:
            continue
        # `phrase` itself is single-word here (already), but allow space
        # boundaries so "stopsign" doesn't accidentally match "stop sign".
        if phrase not in norm.split() and phrase not in (
            f" {norm} "  # quick contains-substring with whitespace guard
        ):
            continue
        # Find tokens that follow the phrase to hunt for an index.
        tokens = norm.split()
        try:
            after = tokens[tokens.index(phrase.split()[-1]) + 1:]
        except ValueError:
            after = []
        return ParsedCommand(semantic_name=canonical, desired_index=_scan_index(after))

    # 2. Direct token-level match against `known_targets`.
    tokens = norm.split()
    for i, tok in enumerate(tokens):
        if tok in _FILLER:
            continue
        if tok in known_targets:
            return ParsedCommand(
                semantic_name=tok,
                desired_index=_scan_index(tokens[i + 1:]),
            )

    return None


# ── Target selection ──────────────────────────────────────────────────────

def select_target(
    objects: list[MemoryObject],
    semantic_name: str,
    detector_label: str,
    query_text: str,
    desired_index: Optional[int] = None,
) -> QueryResult:
    """Select the best memory object matching the given detector_label.

    Two selection modes:

    * **Indexed** (`desired_index is not None`): require an exact
      `object_id == f"{detector_label}_{desired_index}"` match. Memory
      assigns ids as `<detector_label>_<seq>` starting at 0 in observation
      order (see `tb3_memory.memory_core._create_new`), so e.g.
      "go to person 3" maps to `person_3`. If the requested index has
      not been observed yet, return a failure listing the ids that *are*
      available.

    * **Default** (`desired_index is None`): nearest-first selection.
      Picks the candidate with smallest Euclidean distance to the output
      frame origin (which is `base_link` by default, i.e. the robot
      itself), with confidence as the tiebreaker. This matches the user
      intuition "go to the person" → the closest one.
    """
    candidates = [o for o in objects if o.detector_label == detector_label]

    if not candidates:
        return QueryResult(
            success=False,
            query_text=query_text,
            semantic_name=semantic_name,
            detector_label=detector_label,
            status_message=f"no active {semantic_name} in memory",
        )

    # ── Indexed lookup ───────────────────────────────────────────────────
    if desired_index is not None:
        target_id = f"{detector_label}_{desired_index}"
        match = next((o for o in candidates if o.object_id == target_id), None)
        if match is None:
            available = sorted(o.object_id for o in candidates)
            return QueryResult(
                success=False,
                query_text=query_text,
                semantic_name=semantic_name,
                detector_label=detector_label,
                status_message=(
                    f"requested {target_id} not in memory; "
                    f"available {semantic_name}: {available}"
                ),
            )
        return QueryResult(
            success=True,
            query_text=query_text,
            semantic_name=semantic_name,
            detector_label=detector_label,
            object_id=match.object_id,
            x=match.x,
            y=match.y,
            confidence=match.confidence,
            status_message=(
                f"matched {match.object_id} by index "
                f"(d={math.hypot(match.x, match.y):.2f}m, "
                f"{len(candidates)} candidate(s))"
            ),
        )

    # ── Default: nearest-first ───────────────────────────────────────────
    # `min` by (distance, -confidence): smaller distance wins; on ties
    # a higher confidence wins.
    best = min(candidates, key=lambda o: (math.hypot(o.x, o.y), -o.confidence))

    return QueryResult(
        success=True,
        query_text=query_text,
        semantic_name=semantic_name,
        detector_label=detector_label,
        object_id=best.object_id,
        x=best.x,
        y=best.y,
        confidence=best.confidence,
        status_message=(
            f"matched {best.object_id} nearest "
            f"(d={math.hypot(best.x, best.y):.2f}m, "
            f"n={best.confidence:.0f}, "
            f"{len(candidates)} candidate(s))"
        ),
    )
