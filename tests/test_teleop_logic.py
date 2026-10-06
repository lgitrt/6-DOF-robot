import math
import unittest

from ros2 import keyboard_teleop
from ros2.teleop_logic import (
    JOINT_NAMES,
    joint_state_fields,
    update_joint_positions,
)


class TeleopLogicTests(unittest.TestCase):
    def test_node_module_imports_without_ros(self):
        self.assertTrue(hasattr(keyboard_teleop, "KeyboardTeleopNode"))

    def test_joint_state_command_names_and_radians(self):
        names, positions = joint_state_fields([180.0, -90.0, 45.0, 0.0, 30.0, -360.0])
        self.assertEqual(names, JOINT_NAMES)
        self.assertEqual(
            positions,
            [math.pi, -math.pi / 2, math.pi / 4, 0.0, math.pi / 6, -2 * math.pi],
        )

    def test_key_updates_joint_in_degrees_and_obeys_limits(self):
        positions, keep_running, joint_index = update_joint_positions(
            [169.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "q",
            5.0,
        )
        self.assertTrue(keep_running)
        self.assertEqual(joint_index, 0)
        self.assertEqual(positions, [170.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_each_positive_key_targets_its_matching_joint(self):
        for index, key in enumerate("qwerty"):
            with self.subTest(key=key):
                positions, keep_running, joint_index = update_joint_positions(
                    [0.0] * 6, key, 2.0
                )
                self.assertTrue(keep_running)
                self.assertEqual(joint_index, index)
                self.assertEqual(positions[index], 2.0)
                self.assertEqual(sum(positions), 2.0)

    def test_reset_and_quit_commands(self):
        positions, keep_running, joint_index = update_joint_positions(
            [10.0] * 6, " ", 2.0
        )
        self.assertEqual(positions, [0.0] * 6)
        self.assertTrue(keep_running)
        self.assertIsNone(joint_index)

        positions, keep_running, joint_index = update_joint_positions(
            [10.0] * 6, "\x03", 2.0
        )
        self.assertEqual(positions, [10.0] * 6)
        self.assertFalse(keep_running)
        self.assertIsNone(joint_index)

    def test_unknown_key_does_not_change_command(self):
        original = [0.0] * 6
        positions, keep_running, joint_index = update_joint_positions(original, "x", 2.0)
        self.assertEqual(positions, original)
        self.assertTrue(keep_running)
        self.assertIsNone(joint_index)


if __name__ == "__main__":
    unittest.main()
