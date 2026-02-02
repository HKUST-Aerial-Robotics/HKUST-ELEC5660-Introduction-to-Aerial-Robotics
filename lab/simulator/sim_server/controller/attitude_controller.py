"""
Cascaded attitude PID controller matching crazyflie-firmware.

Based on: crazyflie-firmware/src/modules/src/controller/attitude_pid_controller.c

Architecture:
    Attitude PID (outer loop): angle error -> desired rate
    Rate PID (inner loop):     rate error  -> actuator output

The actuator output is in the legacy format: int16 roll, pitch, yaw.
These are then fed to the power distribution / motor mixer.
"""
import torch
from typing import Optional, Tuple

from .pid import PID
from . import config


class AttitudeController:
    """
    Vectorized cascaded attitude PID controller.

    Matches the firmware's attitude_pid_controller.c implementation.
    All angles are in degrees (firmware convention).
    """

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
        dt: float = config.ATTITUDE_UPDATE_DT,
    ):
        """
        Initialize attitude controller.

        Args:
            num_envs: Number of parallel environments
            device: Torch device
            dt: Update time step (default: 1/500 = 2ms)
        """
        self.num_envs = num_envs
        self.device = device
        self.dt = dt

        # --- Outer loop: Attitude PID (angle -> rate) ---
        self.pid_roll = PID(
            num_envs, config.PID_ROLL_KP, config.PID_ROLL_KI, config.PID_ROLL_KD,
            dt, device, config.PID_ROLL_KFF
        )
        self.pid_roll.set_integral_limit(config.PID_ROLL_INTEGRATION_LIMIT)

        self.pid_pitch = PID(
            num_envs, config.PID_PITCH_KP, config.PID_PITCH_KI, config.PID_PITCH_KD,
            dt, device, config.PID_PITCH_KFF
        )
        self.pid_pitch.set_integral_limit(config.PID_PITCH_INTEGRATION_LIMIT)

        self.pid_yaw = PID(
            num_envs, config.PID_YAW_KP, config.PID_YAW_KI, config.PID_YAW_KD,
            dt, device, config.PID_YAW_KFF
        )
        self.pid_yaw.set_integral_limit(config.PID_YAW_INTEGRATION_LIMIT)

        # --- Inner loop: Rate PID (rate -> actuator) ---
        self.pid_roll_rate = PID(
            num_envs, config.PID_ROLL_RATE_KP, config.PID_ROLL_RATE_KI, config.PID_ROLL_RATE_KD,
            dt, device, config.PID_ROLL_RATE_KFF
        )
        self.pid_roll_rate.set_integral_limit(config.PID_ROLL_RATE_INTEGRATION_LIMIT)

        self.pid_pitch_rate = PID(
            num_envs, config.PID_PITCH_RATE_KP, config.PID_PITCH_RATE_KI, config.PID_PITCH_RATE_KD,
            dt, device, config.PID_PITCH_RATE_KFF
        )
        self.pid_pitch_rate.set_integral_limit(config.PID_PITCH_RATE_INTEGRATION_LIMIT)

        self.pid_yaw_rate = PID(
            num_envs, config.PID_YAW_RATE_KP, config.PID_YAW_RATE_KI, config.PID_YAW_RATE_KD,
            dt, device, config.PID_YAW_RATE_KFF
        )
        self.pid_yaw_rate.set_integral_limit(config.PID_YAW_RATE_INTEGRATION_LIMIT)

        # Output storage (int16 equivalent)
        self.roll_output = torch.zeros(num_envs, device=device)
        self.pitch_output = torch.zeros(num_envs, device=device)
        self.yaw_output = torch.zeros(num_envs, device=device)

    def reset(self, attitude: Optional[torch.Tensor] = None, env_ids: Optional[torch.Tensor] = None):
        """
        Reset all PID states.

        Args:
            attitude: Current attitude [num_envs, 3] (roll, pitch, yaw) in degrees
            env_ids: Specific environments to reset
        """
        if attitude is not None:
            roll = attitude[:, 0] if env_ids is None else attitude[env_ids, 0]
            pitch = attitude[:, 1] if env_ids is None else attitude[env_ids, 1]
            yaw = attitude[:, 2] if env_ids is None else attitude[env_ids, 2]
        else:
            roll = pitch = yaw = None

        # Reset attitude PIDs
        self.pid_roll.reset(roll, env_ids)
        self.pid_pitch.reset(pitch, env_ids)
        self.pid_yaw.reset(yaw, env_ids)

        # Reset rate PIDs (with zero initial rate)
        self.pid_roll_rate.reset(None, env_ids)
        self.pid_pitch_rate.reset(None, env_ids)
        self.pid_yaw_rate.reset(None, env_ids)

        # Reset outputs
        if env_ids is None:
            self.roll_output.zero_()
            self.pitch_output.zero_()
            self.yaw_output.zero_()
        else:
            self.roll_output[env_ids] = 0.0
            self.pitch_output[env_ids] = 0.0
            self.yaw_output[env_ids] = 0.0

    def correct_attitude(
        self,
        roll_actual: torch.Tensor,
        pitch_actual: torch.Tensor,
        yaw_actual: torch.Tensor,
        roll_desired: torch.Tensor,
        pitch_desired: torch.Tensor,
        yaw_desired: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Compute desired rates from attitude error (outer loop).

        Matches attitudeControllerCorrectAttitudePID() from firmware.

        Args:
            roll_actual: Current roll angle [num_envs] (degrees)
            pitch_actual: Current pitch angle [num_envs] (degrees)
            yaw_actual: Current yaw angle [num_envs] (degrees)
            roll_desired: Desired roll angle [num_envs] (degrees)
            pitch_desired: Desired pitch angle [num_envs] (degrees)
            yaw_desired: Desired yaw angle [num_envs] (degrees)

        Returns:
            Tuple of (roll_rate_desired, pitch_rate_desired, yaw_rate_desired) in deg/s
        """
        roll_rate_desired = self.pid_roll.update_with_setpoint(roll_desired, roll_actual, False)
        pitch_rate_desired = self.pid_pitch.update_with_setpoint(pitch_desired, pitch_actual, False)
        yaw_rate_desired = self.pid_yaw.update_with_setpoint(yaw_desired, yaw_actual, True)

        # Cache for debugging
        self.last_rate_desired = torch.stack(
            [roll_rate_desired, pitch_rate_desired, yaw_rate_desired], dim=1
        )
        self.last_att_desired = torch.stack(
            [roll_desired, pitch_desired, yaw_desired], dim=1
        )

        return roll_rate_desired, pitch_rate_desired, yaw_rate_desired

    def correct_rate(
        self,
        roll_rate_actual: torch.Tensor,
        pitch_rate_actual: torch.Tensor,
        yaw_rate_actual: torch.Tensor,
        roll_rate_desired: torch.Tensor,
        pitch_rate_desired: torch.Tensor,
        yaw_rate_desired: torch.Tensor,
    ):
        """
        Compute actuator output from rate error (inner loop).

        Matches attitudeControllerCorrectRatePID() from firmware.

        Args:
            roll_rate_actual: Current roll rate [num_envs] (deg/s)
            pitch_rate_actual: Current pitch rate [num_envs] (deg/s)
            yaw_rate_actual: Current yaw rate [num_envs] (deg/s)
            roll_rate_desired: Desired roll rate [num_envs] (deg/s)
            pitch_rate_desired: Desired pitch rate [num_envs] (deg/s)
            yaw_rate_desired: Desired yaw rate [num_envs] (deg/s)
        """
        self.roll_output = self.pid_roll_rate.update_with_setpoint(roll_rate_desired, roll_rate_actual, False)
        self.pitch_output = self.pid_pitch_rate.update_with_setpoint(pitch_rate_desired, pitch_rate_actual, False)
        self.yaw_output = self.pid_yaw_rate.update_with_setpoint(yaw_rate_desired, yaw_rate_actual, False)

        # Saturate to int16 range (firmware does this)
        max_val = 32767.0
        self.roll_output = torch.clamp(self.roll_output, -max_val, max_val)
        self.pitch_output = torch.clamp(self.pitch_output, -max_val, max_val)
        self.yaw_output = torch.clamp(self.yaw_output, -max_val, max_val)

    def get_actuator_output(self) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Get the actuator output.

        Returns:
            Tuple of (roll, pitch, yaw) actuator commands [num_envs] each
        """
        return self.roll_output, self.pitch_output, self.yaw_output

    def update(
        self,
        attitude_actual: torch.Tensor,
        gyro: torch.Tensor,
        attitude_desired: torch.Tensor,
        rate_mode: Optional[torch.Tensor] = None,
        rate_setpoint: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Full cascaded attitude control update.

        Args:
            attitude_actual: Current Euler angles [num_envs, 3] (roll, pitch, yaw) in degrees
            gyro: Current gyro rates [num_envs, 3] (x, y, z) in deg/s
            attitude_desired: Desired Euler angles [num_envs, 3] (roll, pitch, yaw) in degrees
            rate_mode: Optional mask [num_envs, 3] - True to use rate setpoint directly
            rate_setpoint: Optional rate setpoints [num_envs, 3] in deg/s (when rate_mode=True)

        Returns:
            Tuple of (roll, pitch, yaw) actuator commands [num_envs] each
        """
        # Extract components
        roll_actual = attitude_actual[:, 0]
        pitch_actual = attitude_actual[:, 1]
        yaw_actual = attitude_actual[:, 2]

        roll_desired = attitude_desired[:, 0]
        pitch_desired = attitude_desired[:, 1]
        yaw_desired = attitude_desired[:, 2]

        # This handles the body frame convention
        roll_rate_actual = gyro[:, 0]
        pitch_rate_actual = gyro[:, 1]
        yaw_rate_actual = gyro[:, 2]
        self.last_rate_actual = torch.stack(
            [roll_rate_actual, pitch_rate_actual, yaw_rate_actual], dim=1
        )

        # Outer loop: attitude -> rate
        roll_rate_des, pitch_rate_des, yaw_rate_des = self.correct_attitude(
            roll_actual, pitch_actual, yaw_actual,
            roll_desired, pitch_desired, yaw_desired
        )

        # Handle rate mode override (for rate-controlled flight modes)
        if rate_mode is not None and rate_setpoint is not None:
            roll_rate_des = torch.where(rate_mode[:, 0], rate_setpoint[:, 0], roll_rate_des)
            pitch_rate_des = torch.where(rate_mode[:, 1], rate_setpoint[:, 1], pitch_rate_des)
            yaw_rate_des = torch.where(rate_mode[:, 2], rate_setpoint[:, 2], yaw_rate_des)

        # Inner loop: rate -> actuator
        self.correct_rate(
            roll_rate_actual, pitch_rate_actual, yaw_rate_actual,
            roll_rate_des, pitch_rate_des, yaw_rate_des
        )

        return self.get_actuator_output()

    def reset_roll_attitude_pid(self, roll_actual: torch.Tensor, env_ids: Optional[torch.Tensor] = None):
        """Reset only roll attitude PID."""
        self.pid_roll.reset(roll_actual, env_ids)

    def reset_pitch_attitude_pid(self, pitch_actual: torch.Tensor, env_ids: Optional[torch.Tensor] = None):
        """Reset only pitch attitude PID."""
        self.pid_pitch.reset(pitch_actual, env_ids)
