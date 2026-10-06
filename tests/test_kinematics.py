import unittest
from pathlib import Path

import ikpy.chain
import numpy as np

from python_gui.kinematics import (
    build_target_frame,
    rotation_matrix,
    solve_inverse_kinematics,
    validate_target_frame,
)


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        urdf_path = Path(__file__).resolve().parents[1] / "python_gui" / "arm_urdf.urdf"
        cls.chain = ikpy.chain.Chain.from_urdf_file(
            str(urdf_path),
            active_links_mask=[False, True, True, False, True, True, True, True],
        )

    def test_euler_rotation_is_right_handed_and_orthonormal(self):
        rotation = rotation_matrix(35.0, -20.0, 63.0)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.0, places=12)

    def test_euler_rotation_uses_rx_ry_rz_composition_order(self):
        expected = np.array([
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
        ])
        np.testing.assert_allclose(rotation_matrix(90.0, 90.0, 90.0), expected, atol=1e-12)

    def test_gui_target_uses_base_link_axes_and_mm_to_m_conversion(self):
        target = build_target_frame((125.0, -43.0, 210.0), (35.0, -20.0, 63.0))
        np.testing.assert_allclose(target[:3, 3], [0.125, -0.043, 0.210])
        np.testing.assert_allclose(target[:3, :3], rotation_matrix(35.0, -20.0, 63.0))

    def test_reflected_frame_is_rejected(self):
        reflected = np.eye(4)
        reflected[[1, 2], :3] = reflected[[2, 1], :3]
        with self.assertRaisesRegex(ValueError, "proper rotation"):
            validate_target_frame(reflected)

    def test_inverse_kinematics_round_trips_a_forward_kinematics_pose(self):
        joints = np.array([0.0, 0.2, -0.3, 0.0, -0.8, 0.25, 0.1, -0.2])
        target = self.chain.forward_kinematics(joints)
        seed = [0.0, 0.0, 1.57, 0.0, -3.14, 0.0, 0.0, 0.0]

        solution = solve_inverse_kinematics(self.chain, target, seed, optimizer_budget=20)
        actual = self.chain.forward_kinematics(solution)

        np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=1e-6)
        np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
