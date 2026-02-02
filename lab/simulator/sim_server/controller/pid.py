"""
Vectorized PID controller matching crazyflie-firmware implementation.

Based on: crazyflie-firmware/src/utils/src/pid.c

Key features from firmware:
- Derivative on measurement (not error) to avoid derivative kick
- Integral limiting with anti-windup
- Yaw angle wrapping for circular angles
- Output limiting
- Feedforward term
"""
import torch
from typing import Optional


class PID:
    """
    Vectorized PID controller matching crazyflie-firmware behavior.

    The PID computes:
        output = kp * error + ki * integral + kd * deriv + kff * setpoint

    Where deriv is computed on the negative measurement change (not error change)
    to avoid derivative kick on setpoint changes.
    """

    def __init__(
        self,
        num_envs: int,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        device: torch.device,
        kff: float = 0.0,
        i_limit: float = 5000.0,
        output_limit: float = 0.0,
    ):
        """
        Initialize vectorized PID controller.

        Args:
            num_envs: Number of parallel environments
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            dt: Time step (seconds)
            device: Torch device
            kff: Feedforward gain (default 0)
            i_limit: Integral limit (0 = no limit)
            output_limit: Output limit (0 = no limit)
        """
        self.num_envs = num_envs
        self.device = device
        self.dt = dt

        # Gains stored as scalars (applied uniformly across envs)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff

        # Limits
        self.i_limit = i_limit
        self.output_limit = output_limit

        # State tensors [num_envs]
        self.desired = torch.zeros(num_envs, device=device)
        self.error = torch.zeros(num_envs, device=device)
        self.prev_measured = torch.zeros(num_envs, device=device)
        self.integ = torch.zeros(num_envs, device=device)
        self.deriv = torch.zeros(num_envs, device=device)

        # Debug outputs [num_envs]
        self.out_p = torch.zeros(num_envs, device=device)
        self.out_i = torch.zeros(num_envs, device=device)
        self.out_d = torch.zeros(num_envs, device=device)
        self.out_ff = torch.zeros(num_envs, device=device)

    def set_gains(
        self,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        kff: Optional[float] = None,
    ):
        """Update PID gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if kff is not None:
            self.kff = kff

    def set_integral_limit(self, limit: float):
        """Set integral limit for anti-windup."""
        self.i_limit = limit

    def set_output_limit(self, limit: float):
        """Set output saturation limit."""
        self.output_limit = limit

    def reset(self, initial: Optional[torch.Tensor] = None, env_ids: Optional[torch.Tensor] = None):
        """
        Reset PID state.

        Args:
            initial: Initial measurement value [num_envs] or [len(env_ids)]
            env_ids: Specific environment indices to reset (None = all)
        """
        if env_ids is None:
            self.error.zero_()
            self.integ.zero_()
            self.deriv.zero_()
            if initial is not None:
                self.prev_measured = initial.clone()
            else:
                self.prev_measured.zero_()
        else:
            self.error[env_ids] = 0.0
            self.integ[env_ids] = 0.0
            self.deriv[env_ids] = 0.0
            if initial is not None:
                self.prev_measured[env_ids] = initial
            else:
                self.prev_measured[env_ids] = 0.0

    def set_desired(self, desired: torch.Tensor):
        """Set the setpoint."""
        self.desired = desired

    def update(self, measured: torch.Tensor, is_yaw_angle: bool = False) -> torch.Tensor:
        """
        Update PID and compute output.

        Matches firmware pid.c pidUpdate() function.

        Args:
            measured: Current measurement [num_envs]
            is_yaw_angle: True if this is yaw angle PID (handles ±180° wrap)

        Returns:
            PID output [num_envs]
        """
        # Compute error
        self.error = self.desired - measured

        # Handle yaw angle wrapping
        if is_yaw_angle:
            # Wrap error to [-180, 180] degrees
            self.error = torch.where(
                self.error > 180.0,
                self.error - 360.0,
                self.error
            )
            self.error = torch.where(
                self.error < -180.0,
                self.error + 360.0,
                self.error
            )

        # Proportional term
        self.out_p = self.kp * self.error
        output = self.out_p

        # Derivative on measurement (not error) to avoid derivative kick
        # Note: negative because we want derivative of error, and error = desired - measured
        delta = -(measured - self.prev_measured)

        # Handle yaw angle wrapping for derivative
        if is_yaw_angle:
            delta = torch.where(delta > 180.0, delta - 360.0, delta)
            delta = torch.where(delta < -180.0, delta + 360.0, delta)

        self.deriv = delta / self.dt

        # Handle NaN (can occur on first step or discontinuities)
        self.deriv = torch.where(torch.isnan(self.deriv), torch.zeros_like(self.deriv), self.deriv)

        self.out_d = self.kd * self.deriv
        output = output + self.out_d

        # Integral term with anti-windup
        self.integ = self.integ + self.error * self.dt

        if self.i_limit > 0:
            self.integ = torch.clamp(self.integ, -self.i_limit, self.i_limit)

        self.out_i = self.ki * self.integ
        output = output + self.out_i

        # Feedforward term
        self.out_ff = self.kff * self.desired
        output = output + self.out_ff

        # Output limiting
        if self.output_limit > 0:
            output = torch.clamp(output, -self.output_limit, self.output_limit)

        # Store for next derivative calculation
        self.prev_measured = measured.clone()

        return output

    def update_with_setpoint(
        self, setpoint: torch.Tensor, measured: torch.Tensor, is_yaw_angle: bool = False
    ) -> torch.Tensor:
        """
        Convenience method to set desired and update in one call.

        Args:
            setpoint: Desired value [num_envs]
            measured: Current measurement [num_envs]
            is_yaw_angle: True if this is yaw angle PID

        Returns:
            PID output [num_envs]
        """
        self.set_desired(setpoint)
        return self.update(measured, is_yaw_angle)
