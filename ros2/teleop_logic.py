"""ROS-independent keyboard and JointState command logic."""

import math
from collections.abc import Sequence

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Maps each key to (joint_index, direction).
KEY_BINDINGS = {
    "q": (0, +1), "a": (0, -1),
    "w": (1, +1), "s": (1, -1),
    "e": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "t": (4, +1), "g": (4, -1),
    "y": (5, +1), "h": (5, -1),
}

JOINT_LIMITS_DEG = [
    (-170.0, 170.0), (-100.0, 100.0), (-100.0, 100.0),
    (-170.0, 170.0), (-120.0, 120.0), (-360.0, 360.0),
]
RESET_KEY = " "
QUIT_KEYS = {"\x1b", "\x03"}


def update_joint_positions(
    positions_deg: Sequence[float],
    key: str,
    step_deg: float,
) -> tuple[list[float], bool, int | None]:
    """Apply one key to a six-joint degree vector; return vector, continue, index."""
    if len(positions_deg) != len(JOINT_NAMES):
        raise ValueError("Exactly six joint positions are required")

    updated = list(positions_deg)
    if key in QUIT_KEYS:
        return updated, False, None
    if key == RESET_KEY:
        return [0.0] * len(JOINT_NAMES), True, None

    binding = KEY_BINDINGS.get(key.lower())
    if binding is None:
        return updated, True, None

    index, direction = binding
    low, high = JOINT_LIMITS_DEG[index]
    updated[index] = max(low, min(high, updated[index] + direction * step_deg))
    return updated, True, index


def joint_state_fields(positions_deg: Sequence[float]) -> tuple[list[str], list[float]]:
    """Return JointState names and positions, converting degrees to radians."""
    if len(positions_deg) != len(JOINT_NAMES):
        raise ValueError("Exactly six joint positions are required")
    return list(JOINT_NAMES), [math.radians(position) for position in positions_deg]
