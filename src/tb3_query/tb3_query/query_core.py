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

    Raises:
        FileNotFoundError: the file is missing or unreadable. The message
            names the path and the package that ships it, because the usual
            cause is a stale/absent install tree rather than a bad parameter.
        ValueError: the file was read but its contents are not a usable
            registry -- it is not valid YAML, it does not parse to a mapping,
            `semantic_targets` is not a list, or an entry is not a mapping or
            is missing `semantic_name` / `detector_label`.
    """
    path = Path(yaml_path)
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    except OSError as exc:
        raise FileNotFoundError(
            "could not read semantic_targets.yaml at %s (%s). It ships in "
            "tb3_bringup (src/tb3_bringup/config/semantic_targets.yaml). "
            "Rebuild with `colcon build --symlink-install` and re-source "
            "install/setup.bash, or point the semantic_targets_file "
            "parameter at an existing file." % (path, exc)
        ) from exc
    except yaml.YAMLError as exc:
        # A tab, a bad indent or an unclosed quote raises ScannerError /
        # ParserError, neither of which is an OSError. Without this branch the
        # node died on a raw PyYAML traceback that never named the file.
        raise ValueError(
            "%s is not valid YAML (%s). Fix the file, or point the "
            "semantic_targets_file parameter at a good copy." % (path, exc)
        ) from exc

    if not isinstance(data, dict):
        raise ValueError(
            "%s does not parse to a YAML mapping; expected a top-level "
            "`semantic_targets:` list." % path
        )

    targets = data.get("semantic_targets", [])
    if not isinstance(targets, list):
        raise ValueError(
            "%s: `semantic_targets` must be a list, got %s."
            % (path, type(targets).__name__)
        )
    sem2det: dict[str, str] = {}
    det2sem: dict[str, str] = {}
    for i, entry in enumerate(targets):
        # Each entry is student-editable, so a typo here must produce a message
        # that names the offending entry rather than a bare KeyError.
        if not isinstance(entry, dict):
            raise ValueError(
                "%s: semantic_targets[%d] must be a mapping, got %s."
                % (path, i, type(entry).__name__)
            )
        if not entry.get("enabled", True):
            continue
        try:
            sn = entry["semantic_name"]
            dl = entry["detector_label"]
        except KeyError as exc:
            raise ValueError(
                "%s: semantic_targets[%d] is missing required key %s."
                % (path, i, exc)
            ) from exc
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

# Any semantic_name containing an underscore normalizes to TWO tokens
# ("trash_can" -> "trash can"), which the single-token match in
# parse_command() can never hit. Such targets MUST be listed here.
_PHRASE_ALIASES: dict[str, str] = {
    "trash can": "trash_can",
    "trashcan": "trash_can",
    "bin": "trash_can",
    # retired targets, kept harmless: parse_command() skips any alias whose
    # canonical name is absent from semantic_targets.yaml.
    "bench": "table",
    "stop sign": "stop_sign",
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
