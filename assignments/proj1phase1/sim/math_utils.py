from __future__ import annotations

import numpy as np


def wrap_to_pi(angle: np.ndarray | float) -> np.ndarray | float:
    """Wrap angles to [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def quaternion_to_R(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to rotation matrix."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    w2 = w * w
    x2 = x * x
    y2 = y * y
    z2 = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [w2 + x2 - y2 - z2, 2.0 * (xy - wz), 2.0 * (wy + xz)],
            [2.0 * (wz + xy), w2 - x2 + y2 - z2, 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (wx + yz), w2 - x2 - y2 + z2],
        ]
    )


def rot_to_rpy_zxy(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to roll/pitch/yaw (Z-X-Y order)."""
    phi = np.arcsin(R[1, 2])
    psi = np.arctan2(-R[1, 0] / np.cos(phi), R[1, 1] / np.cos(phi))
    theta = np.arctan2(-R[0, 2] / np.cos(phi), R[2, 2] / np.cos(phi))
    return np.array([phi, theta, psi])


def ypr_to_R(ypr: np.ndarray) -> np.ndarray:
    """Convert yaw/pitch/roll to rotation matrix (Rz * Ry * Rx)."""
    y, p, r = ypr
    Rz = np.array(
        [
            [np.cos(y), -np.sin(y), 0.0],
            [np.sin(y), np.cos(y), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    Ry = np.array(
        [
            [np.cos(p), 0.0, np.sin(p)],
            [0.0, 1.0, 0.0],
            [-np.sin(p), 0.0, np.cos(p)],
        ]
    )
    Rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(r), -np.sin(r)],
            [0.0, np.sin(r), np.cos(r)],
        ]
    )
    return Rz @ Ry @ Rx


def R_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to quaternion [w, x, y, z]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q * np.sign(qw) if qw != 0.0 else q
