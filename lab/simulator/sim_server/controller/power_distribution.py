"""
Power distribution (motor mixer) matching crazyflie-firmware.

Based on: crazyflie-firmware/src/modules/src/power_distribution_quadrotor.c

This module handles:
1. Converting control outputs (thrust + roll/pitch/yaw) to motor thrusts
2. Motor capping with attitude priority
3. Converting motor thrusts to body force and torque (for simulator)

Motor layout (X-configuration, viewed from above, body ENU: +X forward, +Y left):
                 Front (+X)
            M4 (CW)     M1 (CCW)
                \\       /
                 \\     /
                  \\   /
                   X  --> +Y (left)
                  / \\
                 /   \\
            M3 (CCW)   M2 (CW)
                 Rear (-X)
"""
import torch
import math
from typing import Tuple, Optional

from . import config


class PowerDistribution:
    """
    Vectorized power distribution matching crazyflie-firmware.

    Supports both legacy mode (int16 roll/pitch/yaw) and force/torque mode.
    Outputs body force and torque for the simulator.
    """

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
    ):
        """
        Initialize power distribution.

        Args:
            num_envs: Number of parallel environments
            device: Torch device
        """
        self.num_envs = num_envs
        self.device = device

        # Physical parameters
        self.arm_length = config.ARM_LENGTH
        self.arm = self.arm_length * math.sqrt(0.5)  # X-config diagonal component
        self.thrust2torque = config.THRUST2TORQUE
        self.thrust_max = config.THRUST_MAX
        self.thrust_min = config.THRUST_MIN
        self.idle_thrust = 0  # Default, can be changed

        # Motor thrust storage [num_envs, 4]
        self.motor_thrust = torch.zeros(num_envs, 4, device=device)

        # Mixing matrix for torque reconstruction (right-hand frame: +X fwd, +Y left, +Z up)
        # Rows: roll, pitch, yaw ; Cols: M1 M2 M3 M4
        self._torque_mix = torch.tensor(
            [
                [-1.0, -1.0, 1.0, 1.0],   # Tx = a*(-F1 - F2 + F3 + F4)
                [-1.0,  1.0, 1.0, -1.0],   # Ty = a*(-F1 + F2 + F3 - F4)  (pitch sign aligned to RH coords)
                [-1.0,  1.0, -1.0, 1.0],   # Tz = k*(-F1 + F2 - F3 + F4)
            ],
            device=device,
        )
        self._torque_scale = torch.tensor([self.arm, self.arm, self.thrust2torque], device=device)

    def set_idle_thrust(self, idle_thrust: int):
        """Set idle thrust (for arming)."""
        self.idle_thrust = idle_thrust

    def legacy_to_motor_thrust(
        self,
        thrust: torch.Tensor,
        roll: torch.Tensor,
        pitch: torch.Tensor,
        yaw: torch.Tensor,
    ) -> torch.Tensor:
        """
        Convert legacy control format to motor thrust.

        Matches powerDistributionLegacy() from crazyflie-firmware for body ENU
        axes (+X forward, +Y left, +Z up) and motor layout:
            M1: front-right (CCW), M2: rear-right (CW),
            M3: rear-left (CCW),  M4: front-left (CW)

        Legacy format:
            - thrust: 0-65535 scale
            - roll/pitch/yaw: int16 (-32767 to 32767)

        Motor mixing for X-configuration:
            M1 = thrust - roll/2 - pitch/2 + yaw
            M2 = thrust - roll/2 + pitch/2 - yaw
            M3 = thrust + roll/2 + pitch/2 + yaw
            M4 = thrust + roll/2 - pitch/2 - yaw

        Args:
            thrust: Collective thrust [num_envs]
            roll: Roll command [num_envs]
            pitch: Pitch command [num_envs]
            yaw: Yaw command [num_envs]

        Returns:
            Motor thrust [num_envs, 4] in 0-65535 scale
        """
        r = roll / 2.0
        p = pitch / 2.0

        # The original firmware has a different sign convention for pitch, so we invert it here for standard right-hand coordinates which is used throughout the isaac-sim based simulator.
        m1 = thrust - r - p + yaw
        m2 = thrust - r + p - yaw
        m3 = thrust + r + p + yaw
        m4 = thrust + r - p - yaw

        return torch.stack([m1, m2, m3, m4], dim=1)

    def force_torque_to_motor_thrust(
        self,
        thrust_si: torch.Tensor,
        torque_x: torch.Tensor,
        torque_y: torch.Tensor,
        torque_z: torch.Tensor,
    ) -> torch.Tensor:
        """
        Convert force/torque to motor thrust.

        Matches powerDistributionForceTorque() from firmware.

        Args:
            thrust_si: Total thrust [num_envs] in Newtons
            torque_x: Roll torque [num_envs] in N*m
            torque_y: Pitch torque [num_envs] in N*m
            torque_z: Yaw torque [num_envs] in N*m

        Returns:
            Motor thrust [num_envs, 4] in 0-65535 scale
        """
        # Convert to per-motor components
        roll_part = 0.25 / self.arm * torque_x
        pitch_part = 0.25 / self.arm * torque_y
        thrust_part = 0.25 * thrust_si  # Per rotor
        yaw_part = 0.25 * torque_z / self.thrust2torque

        # Motor forces (N)
        # Note: Sign convention matches firmware
        m1 = thrust_part - roll_part - pitch_part - yaw_part
        m2 = thrust_part - roll_part + pitch_part + yaw_part
        m3 = thrust_part + roll_part + pitch_part - yaw_part
        m4 = thrust_part + roll_part - pitch_part + yaw_part

        motor_forces = torch.stack([m1, m2, m3, m4], dim=1)

        # Clamp negative forces to zero
        motor_forces = torch.clamp(motor_forces, min=0.0)

        # Convert to PWM scale (0-65535)
        motor_pwm = motor_forces / self.thrust_max * 65535.0

        return motor_pwm

    def cap_thrust(
        self,
        motor_thrust_uncapped: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Cap motor thrust with attitude priority.

        Matches powerDistributionCap() from firmware.
        Reduces all motors uniformly to preserve attitude control.

        Args:
            motor_thrust_uncapped: Uncapped motor thrust [num_envs, 4]

        Returns:
            Tuple of (capped_thrust, is_capped) where:
                - capped_thrust: [num_envs, 4] in 0-65535 scale
                - is_capped: [num_envs] boolean mask
        """
        max_allowed = 65535.0

        # Find highest thrust per environment
        highest, _ = torch.max(motor_thrust_uncapped, dim=1, keepdim=True)

        # Calculate reduction needed
        reduction = torch.clamp(highest - max_allowed, min=0.0)
        is_capped = (reduction > 0).squeeze(-1)

        # Apply uniform reduction to preserve attitude
        capped = motor_thrust_uncapped - reduction

        # Apply minimum thrust (idle thrust)
        capped = torch.clamp(capped, min=float(self.idle_thrust))

        return capped, is_capped

    def motor_thrust_to_force_torque(
        self,
        motor_thrust_pwm: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Convert motor thrust (PWM scale) to body force and torque.

        This is the key function for sim2real - it converts the motor
        commands back to forces for the physics simulation.

        Motor layout (viewed from above, body ENU: +X forward, +Y left):
                 Front (+X)
            M4 (CW)     M1 (CCW)
                \\       /
                 \\     /
                  \\   /
     (left) <-- +Y  X
                  / \\
                 /   \\
            M3 (CCW)   M2 (CW)
                 Rear (-X)

        Args:
            motor_thrust_pwm: Motor thrust [num_envs, 4] in 0-65535 scale

        Returns:
            Tuple of (force, torque) where:
                - force: [num_envs, 3] body frame force (Fx, Fy, Fz) in Newtons
                - torque: [num_envs, 3] body frame torque (Tx, Ty, Tz) in N*m
        """
        # Convert PWM to force (N)
        motor_force = motor_thrust_pwm / 65535.0 * self.thrust_max  # [num_envs, 4]

        # Total thrust (Z-axis in body frame, pointing up)
        total_thrust = torch.sum(motor_force, dim=1)  # [num_envs]

        # Force in body frame (only Z for quadrotor)
        force = torch.zeros(self.num_envs, 3, device=self.device)
        force[:, 2] = total_thrust

        # Torques from motor force differences (right-hand frame, matches legacy_to_motor_thrust pitch inversion)
        torque = (motor_force @ self._torque_mix.transpose(0, 1)) * self._torque_scale

        return force, torque

    def distribute_legacy(
        self,
        thrust: torch.Tensor,
        roll: torch.Tensor,
        pitch: torch.Tensor,
        yaw: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Full legacy power distribution pipeline.

        Takes legacy control commands and returns body force/torque.

        Args:
            thrust: Collective thrust [num_envs] (0-65535)
            roll: Roll command [num_envs] (int16)
            pitch: Pitch command [num_envs] (int16)
            yaw: Yaw command [num_envs] (int16)

        Returns:
            Tuple of (force, torque) for simulator
        """
        # Legacy mixing
        motor_uncapped = self.legacy_to_motor_thrust(thrust, roll, pitch, yaw)

        # Cap with attitude priority
        motor_capped, _ = self.cap_thrust(motor_uncapped)

        # Store for debugging
        self.motor_thrust = motor_capped

        # Convert to force/torque
        return self.motor_thrust_to_force_torque(motor_capped)

    def distribute_force_torque(
        self,
        thrust_si: torch.Tensor,
        torque_x: torch.Tensor,
        torque_y: torch.Tensor,
        torque_z: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Full force/torque power distribution pipeline.

        Takes SI force/torque commands and returns body force/torque
        after motor mixing and capping.

        Args:
            thrust_si: Total thrust [num_envs] in Newtons
            torque_x: Roll torque [num_envs] in N*m
            torque_y: Pitch torque [num_envs] in N*m
            torque_z: Yaw torque [num_envs] in N*m

        Returns:
            Tuple of (force, torque) for simulator
        """
        # Force/torque mixing
        motor_uncapped = self.force_torque_to_motor_thrust(
            thrust_si, torque_x, torque_y, torque_z
        )

        # Cap with attitude priority
        motor_capped, _ = self.cap_thrust(motor_uncapped)

        # Store for debugging
        self.motor_thrust = motor_capped

        # Convert back to force/torque (with capping effects)
        return self.motor_thrust_to_force_torque(motor_capped)

    def thrust_pwm_to_si(self, thrust_pwm: torch.Tensor) -> torch.Tensor:
        """
        Convert thrust from PWM scale to SI (Newtons).

        Args:
            thrust_pwm: Thrust in 0-65535 scale

        Returns:
            Thrust in Newtons (total, not per motor)
        """
        # PWM to normalized [0, 1]
        normalized = thrust_pwm / 65535.0
        # Scale to max thrust (4 motors)
        return normalized * 4.0 * self.thrust_max

    def thrust_si_to_pwm(self, thrust_si: torch.Tensor) -> torch.Tensor:
        """
        Convert thrust from SI (Newtons) to PWM scale.

        Args:
            thrust_si: Total thrust in Newtons

        Returns:
            Thrust in 0-65535 scale
        """
        # Normalize to max thrust (4 motors)
        normalized = thrust_si / (4.0 * self.thrust_max)
        # Clamp to valid range
        normalized = torch.clamp(normalized, 0.0, 1.0)
        # Scale to PWM
        return normalized * 65535.0

    def get_max_thrust_si(self) -> float:
        """Get maximum total thrust in Newtons."""
        return 4.0 * self.thrust_max
