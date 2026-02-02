"""
Main controller module combining all CF2.1 BL firmware-compatible components.

This is the primary entry point for the controller, providing:
1. Cascaded PID control (position -> velocity -> attitude -> rate)
2. Power distribution (motor mixing)
3. Body force/torque output for IsaacLab simulator

The controller is fully vectorized for batch simulation.
"""
import torch
import math
from typing import Tuple, Optional, Dict, Any
from enum import IntEnum

from .attitude_controller import AttitudeController
from .position_controller import PositionController
from .power_distribution import PowerDistribution
from . import config


class ControlMode(IntEnum):
    """Control modes matching firmware."""
    ATTITUDE = 0      # Direct attitude setpoint (roll, pitch, yaw)
    ATTITUDE_RATE = 1  # Direct rate setpoint (roll_rate, pitch_rate, yaw_rate)
    VELOCITY = 2      # Velocity setpoint (vx, vy, vz in body frame)
    POSITION = 3      # Position setpoint (x, y, z in world frame)


class CrazyflieController:
    """
    Complete Crazyflie 2.1 BL firmware-compatible controller.

    Matches the control flow:
        Position PID -> Velocity PID -> Attitude PID -> Rate PID -> Power Distribution

    All units match firmware conventions:
        - Angles in degrees
        - Angular rates in deg/s
        - Position in meters
        - Velocity in m/s
        - Thrust in PWM scale (0-65535) internally, output as Newtons
    """

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
        attitude_dt: float = config.ATTITUDE_UPDATE_DT,
        position_dt: float = config.POSITION_UPDATE_DT,
        enable_debug: bool = False,
    ):
        """
        Initialize the complete controller.

        Args:
            num_envs: Number of parallel environments
            device: Torch device
            attitude_dt: Attitude controller update dt (default 2ms)
            position_dt: Position controller update dt (default 10ms)
        """
        self.num_envs = num_envs
        self.device = device
        self.attitude_dt = attitude_dt
        self.position_dt = position_dt
        self.enable_debug = enable_debug

        # Control mode per environment
        self.control_mode = torch.full(
            (num_envs,), ControlMode.ATTITUDE, dtype=torch.int32, device=device
        )

        # Sub-controllers
        self.attitude_controller = AttitudeController(num_envs, device, attitude_dt)
        self.position_controller = PositionController(num_envs, device, position_dt)
        self.power_distribution = PowerDistribution(num_envs, device)

        # Setpoint storage
        self.attitude_setpoint = torch.zeros(num_envs, 3, device=device)  # roll, pitch, yaw (deg)
        self.rate_setpoint = torch.zeros(num_envs, 3, device=device)  # roll_rate, pitch_rate, yaw_rate (deg/s)
        self.thrust_setpoint = torch.zeros(num_envs, device=device)  # 0-65535 PWM scale
        self.position_setpoint = torch.zeros(num_envs, 3, device=device)  # x, y, z (m)
        self.velocity_setpoint = torch.zeros(num_envs, 3, device=device)  # vx, vy, vz (m/s)

        # Yaw handling for position/velocity mode
        self.yaw_setpoint = torch.zeros(num_envs, device=device)  # Accumulated yaw setpoint
        self.velocity_body = True

        # Step counter for rate limiting
        self.step_counter = 0

        # Cached outer-loop outputs (position/velocity -> attitude/thrust)
        self._outer_roll_cmd = torch.zeros(num_envs, device=device)
        self._outer_pitch_cmd = torch.zeros(num_envs, device=device)
        self._outer_thrust_cmd = torch.zeros(num_envs, device=device)

        # Accumulator to trigger low-rate updates (start "full" so first compute updates immediately)
        self._outer_time_acc = self.position_dt

        # Debug storage for external tools (opt-in to avoid per-step clones)
        self.last_debug = {}

    def reset(
        self,
        state: Optional[Dict[str, torch.Tensor]] = None,
        env_ids: Optional[torch.Tensor] = None,
    ):
        """
        Reset controller state.

        Args:
            state: Current state dict with keys:
                - position: [num_envs, 3]
                - attitude: [num_envs, 3] (roll, pitch, yaw in degrees)
            env_ids: Specific environments to reset (None = all)
        """
        if state is not None:
            attitude = state.get("attitude")
            position = state.get("position")
        else:
            attitude = None
            position = None

        self.attitude_controller.reset(attitude, env_ids)
        self.position_controller.reset(position, env_ids)

        # Reset setpoints and control mode
        if env_ids is None:
            self.control_mode[:] = ControlMode.ATTITUDE
            self.attitude_setpoint.zero_()
            self.rate_setpoint.zero_()
            self.thrust_setpoint.zero_()
            self.position_setpoint.zero_()
            self.velocity_setpoint.zero_()
            self.velocity_body = True
            if attitude is not None:
                self.yaw_setpoint = attitude[:, 2].clone()
            else:
                self.yaw_setpoint.zero_()
            self.step_counter = 0
            self._outer_roll_cmd.zero_()
            self._outer_pitch_cmd.zero_()
            self._outer_thrust_cmd.zero_()
            self._outer_time_acc = self.position_dt
        else:
            self.control_mode[env_ids] = ControlMode.ATTITUDE
            self.attitude_setpoint[env_ids] = 0.0
            self.rate_setpoint[env_ids] = 0.0
            self.thrust_setpoint[env_ids] = 0.0
            self.position_setpoint[env_ids] = 0.0
            self.velocity_setpoint[env_ids] = 0.0
            self.velocity_body = True
            if attitude is not None:
                self.yaw_setpoint[env_ids] = attitude[env_ids, 2]
            else:
                self.yaw_setpoint[env_ids] = 0.0
            self._outer_roll_cmd[env_ids] = 0.0
            self._outer_pitch_cmd[env_ids] = 0.0
            self._outer_thrust_cmd[env_ids] = 0.0

    def set_attitude_setpoint(
        self,
        roll_deg: torch.Tensor,
        pitch_deg: torch.Tensor,
        yaw_deg: torch.Tensor,
        thrust: torch.Tensor,
        env_ids: Optional[torch.Tensor] = None,
    ):
        """
        Set attitude setpoint (physical units).

        Args:
            roll_deg: Roll angle (degrees)
            pitch_deg: Pitch angle (degrees)
            yaw_deg: Yaw angle (degrees)
            thrust: Total thrust (Newtons)
            env_ids: Specific environments (None = all)
        """
        # Clamp to configured limits
        roll_deg = torch.clamp(
            roll_deg,
            -config.ATTITUDE_ROLL_MAX_DEG,
            config.ATTITUDE_ROLL_MAX_DEG,
        )
        pitch_deg = torch.clamp(
            pitch_deg,
            -config.ATTITUDE_PITCH_MAX_DEG,
            config.ATTITUDE_PITCH_MAX_DEG,
        )
        yaw_deg = self._cap_angle(yaw_deg)
        yaw_deg = torch.clamp(
            yaw_deg,
            -config.ATTITUDE_YAW_MAX_DEG,
            config.ATTITUDE_YAW_MAX_DEG,
        )
        thrust = torch.clamp(thrust, 0.0, config.ATTITUDE_THRUST_MAX_N)
        thrust_pwm = self.power_distribution.thrust_si_to_pwm(thrust)

        if env_ids is None:
            self.attitude_setpoint[:, 0] = roll_deg
            self.attitude_setpoint[:, 1] = pitch_deg
            self.yaw_setpoint = yaw_deg
            self.thrust_setpoint = thrust_pwm
            self.control_mode[:] = ControlMode.ATTITUDE
        else:
            self.attitude_setpoint[env_ids, 0] = roll_deg
            self.attitude_setpoint[env_ids, 1] = pitch_deg
            self.yaw_setpoint[env_ids] = yaw_deg
            self.thrust_setpoint[env_ids] = thrust_pwm
            self.control_mode[env_ids] = ControlMode.ATTITUDE

    def set_position_setpoint(
        self,
        x: torch.Tensor,
        y: torch.Tensor,
        z: torch.Tensor,
        yaw: Optional[torch.Tensor] = None,
        env_ids: Optional[torch.Tensor] = None,
    ):
        """
        Set position setpoint.

        Args:
            x: X position (meters)
            y: Y position (meters)
            z: Z position (meters)
            yaw: Yaw angle (degrees), optional
            env_ids: Specific environments (None = all)
        """
        if env_ids is None:
            self.position_setpoint[:, 0] = x
            self.position_setpoint[:, 1] = y
            self.position_setpoint[:, 2] = z
            if yaw is not None:
                yaw = self._cap_angle(yaw)
                yaw = torch.clamp(
                    yaw,
                    -config.ATTITUDE_YAW_MAX_DEG,
                    config.ATTITUDE_YAW_MAX_DEG,
                )
                self.yaw_setpoint = yaw
            self.control_mode[:] = ControlMode.POSITION
        else:
            self.position_setpoint[env_ids, 0] = x
            self.position_setpoint[env_ids, 1] = y
            self.position_setpoint[env_ids, 2] = z
            if yaw is not None:
                yaw = self._cap_angle(yaw)
                yaw = torch.clamp(
                    yaw,
                    -config.ATTITUDE_YAW_MAX_DEG,
                    config.ATTITUDE_YAW_MAX_DEG,
                )
                self.yaw_setpoint[env_ids] = yaw
            self.control_mode[env_ids] = ControlMode.POSITION

    def set_velocity_setpoint(
        self,
        vx: torch.Tensor,
        vy: torch.Tensor,
        vz: torch.Tensor,
        yaw_rate: Optional[torch.Tensor] = None,
        velocity_body: bool = True,
        env_ids: Optional[torch.Tensor] = None,
    ):
        """
        Set velocity setpoint (body frame).

        Args:
            vx: X velocity (m/s, body frame)
            vy: Y velocity (m/s, body frame)
            vz: Z velocity (m/s, world frame)
            yaw_rate: Yaw rate (deg/s), optional
            velocity_body: If True, vx/vy are body-frame (default)
            env_ids: Specific environments (None = all)
        """
        vx = torch.clamp(vx, -config.PID_POS_VEL_X_MAX, config.PID_POS_VEL_X_MAX)
        vy = torch.clamp(vy, -config.PID_POS_VEL_Y_MAX, config.PID_POS_VEL_Y_MAX)
        vz = torch.clamp(vz, -config.PID_POS_VEL_Z_MAX, config.PID_POS_VEL_Z_MAX)
        if yaw_rate is not None:
            yaw_rate = torch.clamp(
                yaw_rate,
                -config.ATTITUDE_YAW_RATE_MAX_DPS,
                config.ATTITUDE_YAW_RATE_MAX_DPS,
            )

        if env_ids is None:
            self.velocity_setpoint[:, 0] = vx
            self.velocity_setpoint[:, 1] = vy
            self.velocity_setpoint[:, 2] = vz
            if yaw_rate is not None:
                self.rate_setpoint[:, 2] = yaw_rate
            self.velocity_body = velocity_body
            self.control_mode[:] = ControlMode.VELOCITY
        else:
            self.velocity_setpoint[env_ids, 0] = vx
            self.velocity_setpoint[env_ids, 1] = vy
            self.velocity_setpoint[env_ids, 2] = vz
            if yaw_rate is not None:
                self.rate_setpoint[env_ids, 2] = yaw_rate
            self.velocity_body = velocity_body
            self.control_mode[env_ids] = ControlMode.VELOCITY

    def _cap_angle(self, angle: torch.Tensor) -> torch.Tensor:
        """Wrap angle to [-180, 180] degrees."""
        return ((angle + 180.0) % 360.0) - 180.0

    def compute(
        self,
        state: Dict[str, torch.Tensor],
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Compute control output (body force and torque).

        Args:
            state: Current state dict with keys:
                - position: [num_envs, 3] world position (m)
                - velocity: [num_envs, 3] world velocity (m/s)
                - attitude: [num_envs, 3] Euler angles (roll, pitch, yaw) in degrees
                - angular_velocity: [num_envs, 3] gyro rates (deg/s)

        Returns:
            Tuple of (force, torque) where:
                - force: [num_envs, 3] body frame force (N)
                - torque: [num_envs, 3] body frame torque (N*m)
        """
        # Extract state
        position = state["position"]
        velocity = state["velocity"]
        attitude = state["attitude"]
        gyro = state["angular_velocity"]

        roll = attitude[:, 0]
        pitch = attitude[:, 1]
        yaw = attitude[:, 2]

        # Low-frequency scheduler for outer loops (position/velocity)
        self._outer_time_acc += self.attitude_dt
        outer_due = False
        if self._outer_time_acc + 1e-9 >= self.position_dt:
            outer_due = True
            self._outer_time_acc -= self.position_dt

        # Initialize attitude and thrust setpoints
        roll_des = self.attitude_setpoint[:, 0].clone()
        pitch_des = self.attitude_setpoint[:, 1].clone()
        yaw_des = self.yaw_setpoint.clone()
        thrust = self.thrust_setpoint.clone()

        # Rate mode flags
        rate_mode = torch.zeros(self.num_envs, 3, dtype=torch.bool, device=self.device)

        # --- Position Control ---
        pos_mode_mask = self.control_mode == ControlMode.POSITION
        if pos_mode_mask.any():
            # Update yaw setpoint from yaw rate
            yaw_rate = self.rate_setpoint[:, 2]
            self.yaw_setpoint = self._cap_angle(
                self.yaw_setpoint + yaw_rate * self.attitude_dt
            )
            yaw_des = self.yaw_setpoint.clone()

            # Position mode - all axes in position control
            mode_position = torch.ones(self.num_envs, 3, dtype=torch.bool, device=self.device)

            if outer_due:
                roll_pos, pitch_pos, thrust_pos = self.position_controller.update(
                    position, velocity, yaw,
                    self.position_setpoint,
                    self.velocity_setpoint,
                    mode_position,
                    velocity_body=False,
                )
                self._outer_roll_cmd = torch.where(pos_mode_mask, roll_pos, self._outer_roll_cmd)
                self._outer_pitch_cmd = torch.where(pos_mode_mask, pitch_pos, self._outer_pitch_cmd)
                self._outer_thrust_cmd = torch.where(pos_mode_mask, thrust_pos, self._outer_thrust_cmd)

            # Apply cached outputs to environments in position mode
            roll_des = torch.where(pos_mode_mask, self._outer_roll_cmd, roll_des)
            pitch_des = torch.where(pos_mode_mask, self._outer_pitch_cmd, pitch_des)
            thrust = torch.where(pos_mode_mask, self._outer_thrust_cmd, thrust)

        # --- Velocity Control ---
        vel_mode_mask = self.control_mode == ControlMode.VELOCITY
        if vel_mode_mask.any():
            # Update yaw setpoint from yaw rate
            yaw_rate = self.rate_setpoint[:, 2]
            self.yaw_setpoint = self._cap_angle(
                self.yaw_setpoint + yaw_rate * self.attitude_dt
            )
            yaw_des = self.yaw_setpoint.clone()

            # Velocity mode - all axes in velocity control
            mode_position = torch.zeros(self.num_envs, 3, dtype=torch.bool, device=self.device)

            if outer_due:
                roll_vel, pitch_vel, thrust_vel = self.position_controller.update(
                    position, velocity, yaw,
                    self.position_setpoint,  # Not used in velocity mode
                    self.velocity_setpoint,
                    mode_position,
                    velocity_body=self.velocity_body,
                )
                self._outer_roll_cmd = torch.where(vel_mode_mask, roll_vel, self._outer_roll_cmd)
                self._outer_pitch_cmd = torch.where(vel_mode_mask, pitch_vel, self._outer_pitch_cmd)
                self._outer_thrust_cmd = torch.where(vel_mode_mask, thrust_vel, self._outer_thrust_cmd)

            roll_des = torch.where(vel_mode_mask, self._outer_roll_cmd, roll_des)
            pitch_des = torch.where(vel_mode_mask, self._outer_pitch_cmd, pitch_des)
            thrust = torch.where(vel_mode_mask, self._outer_thrust_cmd, thrust)

        # --- Attitude Control ---
        att_mode_mask = self.control_mode == ControlMode.ATTITUDE
        if att_mode_mask.any():
            yaw_des = torch.where(att_mode_mask, self.yaw_setpoint, yaw_des)

        # --- Zero thrust handling ---
        # When thrust is zero, disable control (like firmware)
        zero_thrust = thrust == 0
        if zero_thrust.any():
            roll_des = torch.where(zero_thrust, torch.zeros_like(roll_des), roll_des)
            pitch_des = torch.where(zero_thrust, torch.zeros_like(pitch_des), pitch_des)

            # Reset controllers for zero thrust environments
            self.attitude_controller.reset(attitude, torch.where(zero_thrust)[0])
            self.yaw_setpoint = torch.where(zero_thrust, yaw, self.yaw_setpoint)

        # --- Attitude PID ---
        attitude_desired = torch.stack([roll_des, pitch_des, yaw_des], dim=1)

        roll_cmd, pitch_cmd, yaw_cmd = self.attitude_controller.update(
            attitude, gyro, attitude_desired,
            rate_mode, self.rate_setpoint
        )

        # Firmware negates yaw output
        yaw_cmd = -yaw_cmd

        # Zero outputs when thrust is zero
        roll_cmd = torch.where(zero_thrust, torch.zeros_like(roll_cmd), roll_cmd)
        pitch_cmd = torch.where(zero_thrust, torch.zeros_like(pitch_cmd), pitch_cmd)
        yaw_cmd = torch.where(zero_thrust, torch.zeros_like(yaw_cmd), yaw_cmd)
        thrust = torch.where(zero_thrust, torch.zeros_like(thrust), thrust)

        # --- Power Distribution ---
        force, torque = self.power_distribution.distribute_legacy(
            thrust, roll_cmd, pitch_cmd, yaw_cmd
        )

        # Store debug info for external viewers when enabled.
        if self.enable_debug:
            self.last_debug = {
                "attitude_desired": attitude_desired.detach().clone(),
                "attitude": attitude.detach().clone(),
                "gyro": gyro.detach().clone(),
                "rate_desired": getattr(self.attitude_controller, "last_rate_desired", None),
                "rate_actual": getattr(self.attitude_controller, "last_rate_actual", None),
                "roll_cmd": roll_cmd.detach().clone(),
                "pitch_cmd": pitch_cmd.detach().clone(),
                "yaw_cmd": yaw_cmd.detach().clone(),
                "thrust_pwm": thrust.detach().clone(),
                "motor_pwm": self.power_distribution.motor_thrust.detach().clone(),
                "force": force.detach().clone(),
                "torque": torque.detach().clone(),
                "position_setpoint": getattr(self.position_controller, "last_position_setpoint", None),
                "position": getattr(self.position_controller, "last_position", None),
                "velocity_setpoint": getattr(self.position_controller, "last_velocity_setpoint", None),
                "velocity": getattr(self.position_controller, "last_velocity", None),
            }

        self.step_counter += 1

        return force, torque

    def get_debug_info(self) -> Dict[str, Any]:
        """Get debug information about controller state."""
        return {
            "control_mode": self.control_mode.detach().cpu().clone(),
            "attitude_setpoint": self.attitude_setpoint.detach().cpu().clone(),
            "position_setpoint": self.position_setpoint.detach().cpu().clone(),
            "velocity_setpoint": self.velocity_setpoint.detach().cpu().clone(),
            "thrust_setpoint": self.thrust_setpoint.detach().cpu().clone(),
            "yaw_setpoint": self.yaw_setpoint.detach().cpu().clone(),
            "motor_thrust": self.power_distribution.motor_thrust.detach().cpu().clone(),
        }

    def set_gains(self, params: Dict[str, Dict[str, float]]):
        """
        Update PID gains at runtime. Keys supported:
            roll_rate / pitch_rate / yaw_rate / roll / pitch / yaw
            vel_x / vel_y / vel_z
            pos_x / pos_y / pos_z

        Each may include kp, ki, kd, kff, i_limit.
        """
        mapping = {
            "roll_rate": self.attitude_controller.pid_roll_rate,
            "pitch_rate": self.attitude_controller.pid_pitch_rate,
            "yaw_rate": self.attitude_controller.pid_yaw_rate,
            "roll": self.attitude_controller.pid_roll,
            "pitch": self.attitude_controller.pid_pitch,
            "yaw": self.attitude_controller.pid_yaw,
            "vel_x": self.position_controller.pid_vx,
            "vel_y": self.position_controller.pid_vy,
            "vel_z": self.position_controller.pid_vz,
            "pos_x": self.position_controller.pid_x,
            "pos_y": self.position_controller.pid_y,
            "pos_z": self.position_controller.pid_z,
        }
        for name, cfg in params.items():
            pid = mapping.get(name)
            if pid is None or not isinstance(cfg, dict):
                continue
            pid.set_gains(
                kp=cfg.get("kp"),
                ki=cfg.get("ki"),
                kd=cfg.get("kd"),
                kff=cfg.get("kff"),
            )
            if "i_limit" in cfg and cfg["i_limit"] is not None:
                pid.set_integral_limit(cfg["i_limit"])
