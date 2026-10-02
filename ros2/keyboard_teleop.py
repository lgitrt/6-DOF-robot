#!/usr/bin/env python3
"""
keyboard_teleop.py
------------------
Keyboard teleoperation node for the 6-DOF robotic arm.

Reads single key presses from the terminal and publishes incremental joint
position commands as a `sensor_msgs/msg/JointState` message on the
`/joint_commands` topic. The on-board micro-ROS agent running on the
Arduino Mega 2560 subscribes to this topic and drives the six stepper
joints accordingly, so the same message interface is shared with the
teach-in GUI (`qt_gui/content/Screen01.ui.qml`).

Controls
--------
    q / a   -> joint 1  +/-
    w / s   -> joint 2  +/-
    e / d   -> joint 3  +/-
    r / f   -> joint 4  +/-
    t / g   -> joint 5  +/-
    y / h   -> joint 6  +/-
    SPACE   -> reset all joints to the home position
    ESC / Ctrl+C -> quit

Usage
-----
    python3 keyboard_teleop.py --step 2.0 --rate 20

Author: Luca Obwegs
Project: 6-DOF Robotic Arm (https://lucaobwegs.com/6-dof-robotic-arm/)
License: GPL-3.0-only (see LICENSE)
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:  # Allows the key-reading logic to be tested without ROS 2 installed.
    ROS2_AVAILABLE = False

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Maps each key to (joint_index, direction)
KEY_BINDINGS = {
    "q": (0, +1), "a": (0, -1),
    "w": (1, +1), "s": (1, -1),
    "e": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "t": (4, +1), "g": (4, -1),
    "y": (5, +1), "h": (5, -1),
}

JOINT_LIMITS_DEG = [(-170.0, 170.0), (-100.0, 100.0), (-100.0, 100.0),
                     (-170.0, 170.0), (-120.0, 120.0), (-360.0, 360.0)]

RESET_KEY = " "
QUIT_KEYS = {"\x1b", "\x03"}  # ESC, Ctrl+C


def read_key() -> str:
    """Read a single key press from stdin without requiring Enter.

    Uses termios/tty on POSIX systems and msvcrt on Windows.
    """
    if sys.platform.startswith("win"):
        import msvcrt
        return msvcrt.getwch()

    import termios
    import tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def clamp(value: float, limits: Tuple[float, float]) -> float:
    low, high = limits
    return max(low, min(high, value))


class KeyboardTeleopNode(Node):
    """Publishes JointState commands built up from incremental key presses."""

    def __init__(self, step_deg: float, rate_hz: float) -> None:
        super().__init__("keyboard_teleop")
        self.step_deg = step_deg
        self.joint_positions_deg: List[float] = [0.0] * len(JOINT_NAMES)
        self.publisher = self.create_publisher(JointState, "/joint_commands", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_state)

    def apply_key(self, key: str) -> bool:
        """Updates joint targets for a key press. Returns False to request quit."""
        if key in QUIT_KEYS:
            return False
        if key == RESET_KEY:
            self.joint_positions_deg = [0.0] * len(JOINT_NAMES)
            self.get_logger().info("Joints reset to home position")
            return True
        binding = KEY_BINDINGS.get(key.lower())
        if binding is None:
            return True
        index, direction = binding
        new_value = self.joint_positions_deg[index] + direction * self.step_deg
        self.joint_positions_deg[index] = clamp(new_value, JOINT_LIMITS_DEG[index])
        self.get_logger().info(
            f"{JOINT_NAMES[index]}: {self.joint_positions_deg[index]:.1f} deg"
        )
        return True

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self.joint_positions_deg)
        self.publisher.publish(msg)


def print_help() -> None:
    print(__doc__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Keyboard teleoperation for the 6-DOF robotic arm")
    parser.add_argument("--step", type=float, default=2.0, help="Degrees added/removed per key press")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz")
    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("rclpy/sensor_msgs were not found. Install ROS 2 and source the workspace "
              "(e.g. 'source /opt/ros/humble/setup.bash') before running this node.")
        sys.exit(1)

    print_help()

    rclpy.init()
    node = KeyboardTeleopNode(step_deg=args.step, rate_hz=args.rate)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if not node.apply_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
