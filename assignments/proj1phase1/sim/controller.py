from __future__ import annotations

import numpy as np

from .math_utils import quaternion_to_R, rot_to_rpy_zxy, wrap_to_pi
from .model import QuadParams


class Controller:
    """Controller for quadrotor trajectory tracking."""

    def __init__(self, params: QuadParams) -> None:
        """Initialize controller with quadrotor parameters.

        Args:
            params: Quadrotor physical parameters (mass, inertia, etc.)
        """
        self.params = params

        # TODO: Tune these gains for better performance, if needed you can add more parameters
        self.Kp_pos = np.array([0.0, 0.0, 0.0])
        self.Kd_pos = np.array([0.0, 0.0, 0.0])
        self.Kp_angle = np.array([0.0, 0.0, 0.0])
        self.Kd_angle = np.array([0.0, 0.0, 0.0])

    def reset(self) -> None:
        """Reset controller state (if needed)."""
        pass

    def __call__(self, t: float, s: np.ndarray, s_des: np.ndarray) -> tuple[float, np.ndarray]:
        """Compute control outputs (thrust and moments) for the quadrotor.

        Args:
            t: Current time [s]
            s: Current state vector (13,)
               [position, velocity, quaternion, angular_velocity]
            s_des: Desired state vector (11,)
               [position, velocity, acceleration, yaw, yaw_rate]

        Returns:
            F: Total thrust force [N]
            M: Moment vector [Mx, My, Mz] in body frame [N⋅m]
        """
        # Extract quadrotor parameters
        m = self.params.mass
        g = self.params.grav
        I = self.params.I

        # ========================================================================
        # TODO: Implement your controller here
        # ========================================================================


        # ========================================================================
        # End of your implementation
        # ========================================================================

        return F, M
