from __future__ import annotations

import numpy as np


def hover_trajectory(t: float, true_s: np.ndarray) -> np.ndarray:
    """Hover at the origin."""
    return np.zeros(11)


def circle_trajectory(t: float, true_s: np.ndarray) -> np.ndarray:
    """Helix in the xy-plane with growing radius."""
    s_des = np.zeros(11)
    omega = 25.0

    angle = t * omega / 180.0 * np.pi
    x_des = 4.0 * np.cos(angle) * t / omega
    y_des = 4.0 * np.sin(angle) * t / omega
    z_des = 3.0 / 25.0 * t

    x_vdes = (4.0 * np.cos(angle) - omega / 180.0 * np.pi * 4.0 * np.sin(angle) * t) / omega
    y_vdes = (4.0 * np.sin(angle) + omega / 180.0 * np.pi * 4.0 * np.cos(angle) * t) / omega
    z_vdes = 3.0 / 25.0

    x_ades = (
        -2.0 * omega / 180.0 * np.pi * 4.0 * np.sin(angle)
        - omega / 180.0 * np.pi * omega / 180.0 * np.pi * 4.0 * np.cos(angle) * t
    ) / omega
    y_ades = (
        2.0 * omega / 180.0 * np.pi * 4.0 * np.cos(angle)
        - omega / 180.0 * np.pi * omega / 180.0 * np.pi * 4.0 * np.sin(angle) * t
    ) / omega
    z_ades = 0.0

    yaw_des = np.mod(0.1 * np.pi * t, 2.0 * np.pi)
    dyaw_des = 0.1 * np.pi

    s_des[0:3] = [x_des, y_des, z_des]
    s_des[3:6] = [x_vdes, y_vdes, z_vdes]
    s_des[6:9] = [x_ades, y_ades, z_ades]
    s_des[9] = yaw_des
    s_des[10] = dyaw_des
    return s_des


def square_trajectory(t: float, true_s: np.ndarray) -> np.ndarray:
    """Piecewise linear trajectory through the five waypoints."""
    s_des = np.zeros(11)
    omega = 25.0
    corner_t = np.array([0.0, 0.25, 0.5, 0.75, 1.0]) * omega

    corner_x = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    corner_y = np.array([0.0, 2.0, 2.0, 0.0, 0.0])
    corner_z = np.array([0.0, 0.0, 2.0, 2.0, 0.0])

    t = min(t, corner_t[-1])

    x_des = corner_x[-1]
    y_des = corner_y[-1]
    z_des = corner_z[-1]
    x_vdes = 0.0
    y_vdes = 0.0
    z_vdes = 0.0

    for i in range(1, len(corner_t)):
        if t <= corner_t[i] + 1e-8:
            dt = corner_t[i] - corner_t[i - 1]
            x_vdes = (corner_x[i] - corner_x[i - 1]) / dt
            y_vdes = (corner_y[i] - corner_y[i - 1]) / dt
            z_vdes = (corner_z[i] - corner_z[i - 1]) / dt
            ratio = (t - corner_t[i - 1]) / dt
            x_des = corner_x[i - 1] + ratio * (corner_x[i] - corner_x[i - 1])
            y_des = corner_y[i - 1] + ratio * (corner_y[i] - corner_y[i - 1])
            z_des = corner_z[i - 1] + ratio * (corner_z[i] - corner_z[i - 1])
            break

    x_ades = 0.0
    y_ades = 0.0
    z_ades = 0.0

    yaw_des = np.mod(0.2 * np.pi * t, 2.0 * np.pi)
    dyaw_des = 0.2 * np.pi

    s_des[0:3] = [x_des, y_des, z_des]
    s_des[3:6] = [x_vdes, y_vdes, z_vdes]
    s_des[6:9] = [x_ades, y_ades, z_ades]
    s_des[9] = yaw_des
    s_des[10] = dyaw_des
    return s_des
