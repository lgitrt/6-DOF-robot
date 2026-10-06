"""Pose construction and inverse kinematics helpers for the URDF base frame."""

from collections.abc import Sequence

import ikpy.chain
import numpy as np


def rotation_matrix(phi_deg: float, theta_deg: float, psi_deg: float) -> np.ndarray:
    """Return Rx(phi) @ Ry(theta) @ Rz(psi), with angles in degrees."""
    phi, theta, psi = np.deg2rad([phi_deg, theta_deg, psi_deg])
    cphi, ctheta, cpsi = np.cos([phi, theta, psi])
    sphi, stheta, spsi = np.sin([phi, theta, psi])

    rotate_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cphi, -sphi],
        [0.0, sphi, cphi],
    ])
    rotate_y = np.array([
        [ctheta, 0.0, stheta],
        [0.0, 1.0, 0.0],
        [-stheta, 0.0, ctheta],
    ])
    rotate_z = np.array([
        [cpsi, -spsi, 0.0],
        [spsi, cpsi, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rotate_x @ rotate_y @ rotate_z


def validate_target_frame(target_frame: Sequence[Sequence[float]]) -> np.ndarray:
    """Return a homogeneous right-handed target frame, or raise ValueError."""
    frame = np.asarray(target_frame, dtype=float)
    if frame.shape != (4, 4) or not np.isfinite(frame).all():
        raise ValueError("Target frame must be a finite 4x4 matrix")

    rotation = frame[:3, :3]
    if not (
        np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        and np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)
        and np.allclose(frame[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)
    ):
        raise ValueError("Target frame must contain a proper rotation and homogeneous row")
    return frame


def build_target_frame(
    position_mm: Sequence[float],
    orientation_deg: Sequence[float],
) -> np.ndarray:
    """Build a pose in the URDF base_link frame from mm and degree inputs."""
    position = np.asarray(position_mm, dtype=float)
    orientation = np.asarray(orientation_deg, dtype=float)
    if position.shape != (3,) or orientation.shape != (3,):
        raise ValueError("Position and orientation must each contain three values")
    if not np.isfinite(position).all() or not np.isfinite(orientation).all():
        raise ValueError("Position and orientation values must be finite")

    frame = np.eye(4)
    frame[:3, :3] = rotation_matrix(*orientation)
    frame[:3, 3] = position / 1000.0
    return validate_target_frame(frame)


def solve_inverse_kinematics(
    chain: ikpy.chain.Chain,
    target_frame: Sequence[Sequence[float]],
    initial_position: Sequence[float],
    optimizer_budget: int,
) -> np.ndarray:
    """Solve a validated pose in the chain's right-handed base_link frame."""
    frame = validate_target_frame(target_frame)
    if optimizer_budget < 1:
        raise ValueError("optimizer_budget must be at least 1")
    return chain.inverse_kinematics_frame(
        frame,
        initial_position,
        orientation_mode="all",
        optimizer_budget=optimizer_budget,
    )
