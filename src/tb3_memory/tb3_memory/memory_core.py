#!/usr/bin/env python3
"""
memory_core.py — Pure-Python semantic object registry for Stage-3 memory.

Maintains an in-memory dict of ``SemanticObject`` instances, each representing
a unique real-world object the robot has observed.  New observations are either
matched to an existing entry (same label, close enough spatially) or create a
fresh entry.

Pipeline per observation:
    (label, confidence, x, y, timestamp)
        → match against registry by label + distance
        → update existing  OR  create new
        → periodic aging marks stale / removes forgotten objects

No ROS dependency — all time values are plain floats (seconds).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SemanticObject:
    """One remembered semantic object."""

    object_id: str
    detector_label: str
    x: float
    y: float
    frame_id: str
    avg_confidence: float
    times_seen: int
    last_seen: float          # timestamp in seconds
    active: bool = True


@dataclass
class Observation:
    """A single incoming localized detection."""

    detector_label: str
    confidence: float
    x: float
    y: float
    frame_id: str
    timestamp: float          # seconds


class MemoryCore:
    """In-memory semantic object registry with matching, update, and aging."""

    def __init__(
        self,
        match_distance_threshold: float = 1.0,
        position_smoothing_alpha: float = 0.3,
        stale_timeout: float = 5.0,
        remove_timeout: float = 30.0,
    ) -> None:
        self.match_distance_threshold = match_distance_threshold
        self.alpha = position_smoothing_alpha
        self.stale_timeout = stale_timeout
        self.remove_timeout = remove_timeout

        self._objects: dict[str, SemanticObject] = {}
        self._next_id: dict[str, int] = {}       # per-label sequence counter

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(self, obs: Observation) -> SemanticObject:
        """Integrate one observation: match or create, then return the object."""
        matched = self._find_match(obs)
        if matched is not None:
            self._update_existing(matched, obs)
            return matched
        return self._create_new(obs)

    def age(self, current_time: float) -> None:
        """Mark stale objects and remove forgotten ones."""
        to_remove: list[str] = []
        for oid, obj in self._objects.items():
            dt = current_time - obj.last_seen
            if dt > self.remove_timeout:
                to_remove.append(oid)
            elif dt > self.stale_timeout:
                obj.active = False
        for oid in to_remove:
            del self._objects[oid]

    def get_active_objects(self) -> list[SemanticObject]:
        """Return a snapshot of all currently active objects."""
        return [o for o in self._objects.values() if o.active]

    def get_all_objects(self) -> list[SemanticObject]:
        """Return a snapshot of all objects (active + stale)."""
        return list(self._objects.values())

    @property
    def size(self) -> int:
        return len(self._objects)

    # ------------------------------------------------------------------
    # Matching
    # ------------------------------------------------------------------

    def _find_match(self, obs: Observation) -> Optional[SemanticObject]:
        """Find the closest existing object with the same label within threshold."""
        best: Optional[SemanticObject] = None
        best_dist = float("inf")

        for obj in self._objects.values():
            if obj.detector_label != obs.detector_label:
                continue
            d = math.hypot(obj.x - obs.x, obj.y - obs.y)
            if d < self.match_distance_threshold and d < best_dist:
                best = obj
                best_dist = d

        return best

    # ------------------------------------------------------------------
    # Update / Create
    # ------------------------------------------------------------------

    def _update_existing(self, obj: SemanticObject, obs: Observation) -> None:
        a = self.alpha
        obj.x = a * obs.x + (1.0 - a) * obj.x
        obj.y = a * obs.y + (1.0 - a) * obj.y
        obj.avg_confidence = (
            (obj.avg_confidence * obj.times_seen + obs.confidence)
            / (obj.times_seen + 1)
        )
        obj.times_seen += 1
        obj.last_seen = obs.timestamp
        obj.active = True

    def _create_new(self, obs: Observation) -> SemanticObject:
        seq = self._next_id.get(obs.detector_label, 0)
        self._next_id[obs.detector_label] = seq + 1
        oid = f"{obs.detector_label}_{seq}"

        obj = SemanticObject(
            object_id=oid,
            detector_label=obs.detector_label,
            x=obs.x,
            y=obs.y,
            frame_id=obs.frame_id,
            avg_confidence=obs.confidence,
            times_seen=1,
            last_seen=obs.timestamp,
            active=True,
        )
        self._objects[oid] = obj
        return obj
