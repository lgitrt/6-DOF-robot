#!/usr/bin/env python3
"""
keyboard_teleop.py
------------------
Keyboard teleoperation node for the 6-DOF robotic arm.

Reads single key presses from the terminal and publishes incremental joint
position commands as a `sensor_msgs/msg/JointState` message on the
`/joint_commands` topic. JointState positions are published in radians.
This repository contains the publisher only; it does not include a ROS 2
subscriber or a micro-ROS bridge to the Arduino firmware.

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
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:  # Allows the key-reading logic to be tested without ROS 2 installed.
    ROS2_AVAILABLE = False
else:
    ROS2_AVAILABLE = True

if __package__:
    from .teleop_logic import JOINT_NAMES, RESET_KEY, joint_state_fields, update_joint_positions
else:
    from teleop_logic import JOINT_NAMES, RESET_KEY, joint_state_fields, update_joint_positions


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


if ROS2_AVAILABLE:
    class KeyboardTeleopNode(Node):
        """Publishes JointState commands built up from incremental key presses."""

        def __init__(self, step_deg: float, rate_hz: float) -> None:
            super().__init__("keyboard_teleop")
            self.step_deg = step_deg
            self.joint_positions_deg: List[float] = [0.0] * len(JOINT_NAMES)
            self.publisher = self.create_publisher(JointState, "/joint_commands", 10)
            self.timer = self.create_timer(1.0 / rate_hz, self._publish_state)

        def apply_key(self, key: str) -> bool:
            """Update the targets and return False when the user requests quit."""
            self.joint_positions_deg, keep_running, joint_index = update_joint_positions(
                self.joint_positions_deg,
                key,
                self.step_deg,
            )
            if not keep_running:
                return False
            if key == RESET_KEY:
                self.get_logger().info("Joints reset to home position")
            elif joint_index is not None:
                self.get_logger().info(
                    f"{JOINT_NAMES[joint_index]}: "
                    f"{self.joint_positions_deg[joint_index]:.1f} deg"
                )
            return True

        def _publish_state(self) -> None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position = joint_state_fields(self.joint_positions_deg)
            self.publisher.publish(msg)
else:
    class KeyboardTeleopNode:
        def __init__(self, step_deg: float, rate_hz: float) -> None:
            raise RuntimeError("Install and source ROS 2 before creating the teleoperation node")


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
