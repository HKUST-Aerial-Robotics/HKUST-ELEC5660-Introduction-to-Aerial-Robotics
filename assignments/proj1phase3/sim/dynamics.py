from __future__ import annotations

import numpy as np

from .math_utils import quaternion_to_R, rot_to_rpy_zxy
from .model import QuadParams


def quad_eom(
    t: float,
    true_state: np.ndarray,
    F: float,
    M: np.ndarray,
    Fd: np.ndarray,
    params: QuadParams,
) -> np.ndarray:
    """Equations of motion for the quadrotor."""
    xdot = true_state[3]
    ydot = true_state[4]
    zdot = true_state[5]
    qW = true_state[6]
    qX = true_state[7]
    qY = true_state[8]
    qZ = true_state[9]
    p = true_state[10]
    q = true_state[11]
    r = true_state[12]

    Rot = quaternion_to_R(np.array([qW, qX, qY, qZ]))
    phi, theta, yawangle = rot_to_rpy_zxy(Rot)

    BRW = np.array(
        [
            [
                np.cos(yawangle) * np.cos(theta)
                - np.sin(phi) * np.sin(yawangle) * np.sin(theta),
                np.cos(theta) * np.sin(yawangle)
                + np.cos(yawangle) * np.sin(phi) * np.sin(theta),
                -np.cos(phi) * np.sin(theta),
            ],
            [-np.cos(phi) * np.sin(yawangle), np.cos(phi) * np.cos(yawangle), np.sin(phi)],
            [
                np.cos(yawangle) * np.sin(theta)
                + np.cos(theta) * np.sin(phi) * np.sin(yawangle),
                np.sin(yawangle) * np.sin(theta)
                - np.cos(yawangle) * np.cos(theta) * np.sin(phi),
                np.cos(phi) * np.cos(theta),
            ],
        ]
    )
    WRB = BRW.T

    accel = (WRB @ (np.array([0.0, 0.0, F]) + Fd) - np.array([0.0, 0.0, params.mass * params.grav])) / params.mass

    K_quat = 2.0
    quaterror = 1.0 - (qW * qW + qX * qX + qY * qY + qZ * qZ)
    qdot = -0.5 * np.array(
        [
            [0.0, -p, -q, -r],
            [p, 0.0, -r, q],
            [q, r, 0.0, -p],
            [r, -q, p, 0.0],
        ]
    ) @ np.array([qW, qX, qY, qZ]) + K_quat * quaterror * np.array([qW, qX, qY, qZ])

    omega = np.array([p, q, r])
    pqrdot = params.invI @ (M - np.cross(omega, params.I @ omega))

    sdot = np.zeros(13)
    sdot[0] = xdot
    sdot[1] = ydot
    sdot[2] = zdot
    sdot[3:6] = accel
    sdot[6:10] = qdot
    sdot[10:13] = pqrdot
    return sdot
