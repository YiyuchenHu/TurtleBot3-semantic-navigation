#!/usr/bin/env python3
"""
goal_adapter_core.py — Pure math for Stage-5 nav goal adapter.

Converts a target object position into a safe approach pose:
    (target_x, target_y)  →  (goal_x, goal_y, goal_yaw)

The approach pose is offset back from the target along the robot-to-target
direction so the robot stops *approach_distance* metres away and faces
the object.

No ROS dependency.
"""

from __future__ import annotations

import math
from typing import Optional


def compute_approach_pose(
    tx: float,
    ty: float,
    approach_distance: float = 0.5,
    min_standoff: float = 0.3,
) -> Optional[tuple[float, float, float]]:
    """Compute a safe approach (goal_x, goal_y, yaw) from a target position.

    The target is assumed to be in the robot body frame (base_link),
    where the robot is at the origin facing +X.

    Returns None if the target is closer than *min_standoff*.
    """
    dist = math.hypot(tx, ty)

    if dist < min_standoff:
        return None

    direction = math.atan2(ty, tx)

    actual_offset = min(approach_distance, dist - min_standoff)

    gx = tx - actual_offset * math.cos(direction)
    gy = ty - actual_offset * math.sin(direction)
    yaw = direction

    return gx, gy, yaw


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle (rad) to a quaternion (x, y, z, w)."""
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )
