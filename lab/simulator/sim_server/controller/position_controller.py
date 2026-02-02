"""
Cascaded position PID controller matching crazyflie-firmware.

Based on: crazyflie-firmware/src/modules/src/controller/position_controller_pid.c

Architecture:
    Position PID:  position error -> desired velocity
    Velocity PID:  velocity error -> desired attitude + thrust

The position/velocity control is done in body-yaw-aligned frame (body X/Y, world Z).
"""
import torch
import math
from typing import Tuple, Optional

from .pid import PID
from . import config


class PositionController:
    """
    Vectorized cascaded position PID controller.

    Matches the firmware's position_controller_pid.c implementation.
    Uses body-yaw-aligned frame for X/Y control, world frame for Z.
    """

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
        dt: float = config.POSITION_UPDATE_DT,
    ):
        """
        Initialize position controller.

        Args:
            num_envs: Number of parallel environments
            device: Torch device
            dt: Update time step (default: 1/100 = 10ms)
        """
        self.num_envs = num_envs
        self.device = device
        self.dt = dt

        # Velocity limits
        self.x_vel_max = config.PID_POS_VEL_X_MAX
        self.y_vel_max = config.PID_POS_VEL_Y_MAX
        self.z_vel_max = config.PID_POS_VEL_Z_MAX
        self.vel_max_overhead = 1.10  # Firmware uses 1.10x overhead

        # Attitude limits
        self.roll_limit = config.PID_VEL_ROLL_MAX
        self.pitch_limit = config.PID_VEL_PITCH_MAX
        self.rp_limit_overhead = 1.10  # Firmware uses 1.10x overhead

        # Thrust parameters
        self.thrust_base = config.PID_VEL_THRUST_BASE
        self.thrust_min = config.PID_VEL_THRUST_MIN
        self.thrust_scale = 1000.0  # Firmware scale factor

        # --- Position PIDs ---
        self.pid_x = PID(
            num_envs, config.PID_POS_X_KP, config.PID_POS_X_KI, config.PID_POS_X_KD,
            dt, device, config.PID_POS_X_KFF
        )
        self.pid_y = PID(
            num_envs, config.PID_POS_Y_KP, config.PID_POS_Y_KI, config.PID_POS_Y_KD,
            dt, device, config.PID_POS_Y_KFF
        )
        self.pid_z = PID(
            num_envs, config.PID_POS_Z_KP, config.PID_POS_Z_KI, config.PID_POS_Z_KD,
            dt, device, config.PID_POS_Z_KFF
        )

        # --- Velocity PIDs ---
        self.pid_vx = PID(
            num_envs, config.PID_VEL_X_KP, config.PID_VEL_X_KI, config.PID_VEL_X_KD,
            dt, device, config.PID_VEL_X_KFF
        )
        self.pid_vy = PID(
            num_envs, config.PID_VEL_Y_KP, config.PID_VEL_Y_KI, config.PID_VEL_Y_KD,
            dt, device, config.PID_VEL_Y_KFF
        )
        self.pid_vz = PID(
            num_envs, config.PID_VEL_Z_KP, config.PID_VEL_Z_KI, config.PID_VEL_Z_KD,
            dt, device, config.PID_VEL_Z_KFF
        )

    def reset(
        self,
        position: Optional[torch.Tensor] = None,
        env_ids: Optional[torch.Tensor] = None,
    ):
        """
        Reset all PID states.

        Args:
            position: Current position [num_envs, 3] (x, y, z) in meters
            env_ids: Specific environments to reset
        """
        if position is not None:
            if env_ids is not None:
                px, py, pz = position[env_ids, 0], position[env_ids, 1], position[env_ids, 2]
            else:
                px, py, pz = position[:, 0], position[:, 1], position[:, 2]
        else:
            px = py = pz = None

        self.pid_x.reset(px, env_ids)
        self.pid_y.reset(py, env_ids)
        self.pid_z.reset(pz, env_ids)
        self.pid_vx.reset(None, env_ids)
        self.pid_vy.reset(None, env_ids)
        self.pid_vz.reset(None, env_ids)

    def _world_to_body_xy(
        self,
        world_x: torch.Tensor,
        world_y: torch.Tensor,
        yaw: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Transform world X/Y to body-yaw-aligned frame.

        Args:
            world_x: World X coordinate [num_envs]
            world_y: World Y coordinate [num_envs]
            yaw: Current yaw angle [num_envs] in degrees

        Returns:
            Tuple of (body_x, body_y)
        """
        yaw_rad = yaw * (math.pi / 180.0)
        cos_yaw = torch.cos(yaw_rad)
        sin_yaw = torch.sin(yaw_rad)

        body_x = world_x * cos_yaw + world_y * sin_yaw
        body_y = -world_x * sin_yaw + world_y * cos_yaw

        return body_x, body_y

    def position_control(
        self,
        position: torch.Tensor,
        velocity: torch.Tensor,
        yaw: torch.Tensor,
        setpoint_position: torch.Tensor,
        setpoint_velocity: torch.Tensor,
        mode_x: torch.Tensor,
        mode_y: torch.Tensor,
        mode_z: torch.Tensor,
        velocity_body: bool = False,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Compute velocity setpoints from position error.

        Matches positionController() from firmware.

        Args:
            position: Current position [num_envs, 3] (x, y, z) in meters
            velocity: Current velocity [num_envs, 3] (vx, vy, vz) in m/s
            yaw: Current yaw angle [num_envs] in degrees
            setpoint_position: Desired position [num_envs, 3] in meters
            setpoint_velocity: Desired velocity [num_envs, 3] in m/s (feedforward or velocity mode)
            mode_x: Position control mode for X (True = position control)
            mode_y: Position control mode for Y (True = position control)
            mode_z: Position control mode for Z (True = position control)
            velocity_body: If True, setpoint_velocity is in body frame

        Returns:
            Tuple of (vel_x_setpoint, vel_y_setpoint, vel_z_setpoint) in body frame
        """
        # Set output limits
        self.pid_x.set_output_limit(self.x_vel_max * self.vel_max_overhead)
        self.pid_y.set_output_limit(self.y_vel_max * self.vel_max_overhead)
        self.pid_z.set_output_limit(max(self.z_vel_max, 0.5) * self.vel_max_overhead)

        # Transform to body-yaw-aligned frame
        setp_body_x, setp_body_y = self._world_to_body_xy(
            setpoint_position[:, 0], setpoint_position[:, 1], yaw
        )
        state_body_x, state_body_y = self._world_to_body_xy(
            position[:, 0], position[:, 1], yaw
        )

        # Global velocity for feedforward
        global_vx = setpoint_velocity[:, 0]
        global_vy = setpoint_velocity[:, 1]

        # Initialize velocity setpoints
        vel_setpoint_x = setpoint_velocity[:, 0]
        vel_setpoint_y = setpoint_velocity[:, 1]
        vel_setpoint_z = setpoint_velocity[:, 2]

        # X position control
        pos_vel_x = self.pid_x.update_with_setpoint(setp_body_x, state_body_x, False)
        vel_setpoint_x = torch.where(mode_x, pos_vel_x, vel_setpoint_x)

        # If not position mode and not body velocity, transform to body
        if not velocity_body:
            vel_body_x, _ = self._world_to_body_xy(global_vx, global_vy, yaw)
            vel_setpoint_x = torch.where(~mode_x, vel_body_x, vel_setpoint_x)

        # Y position control
        pos_vel_y = self.pid_y.update_with_setpoint(setp_body_y, state_body_y, False)
        vel_setpoint_y = torch.where(mode_y, pos_vel_y, vel_setpoint_y)

        if not velocity_body:
            _, vel_body_y = self._world_to_body_xy(global_vx, global_vy, yaw)
            vel_setpoint_y = torch.where(~mode_y, vel_body_y, vel_setpoint_y)

        # Z position control
        pos_vel_z = self.pid_z.update_with_setpoint(setpoint_position[:, 2], position[:, 2], False)
        vel_setpoint_z = torch.where(mode_z, pos_vel_z, vel_setpoint_z)

        # Cache for debugging
        self.last_position_setpoint = setpoint_position.clone()
        self.last_position_body_setpoint = torch.stack([setp_body_x, setp_body_y, setpoint_position[:, 2]], dim=1)
        self.last_velocity_setpoint = torch.stack([vel_setpoint_x, vel_setpoint_y, vel_setpoint_z], dim=1)
        self.last_position = position.clone()
        self.last_velocity = velocity.clone()
        self.last_yaw = yaw.clone()

        return vel_setpoint_x, vel_setpoint_y, vel_setpoint_z

    def velocity_control(
        self,
        velocity: torch.Tensor,
        yaw: torch.Tensor,
        setpoint_velocity: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Compute attitude and thrust from velocity error.

        Matches velocityController() from firmware.

        Args:
            velocity: Current velocity [num_envs, 3] (vx, vy, vz) in m/s (world frame)
            yaw: Current yaw angle [num_envs] in degrees
            setpoint_velocity: Desired velocity [num_envs, 3] (vx, vy, vz) in body frame

        Returns:
            Tuple of (roll_desired, pitch_desired, thrust) where:
                - roll_desired: degrees
                - pitch_desired: degrees
                - thrust: 0-65535 scale
        """
        # Set output limits
        self.pid_vx.set_output_limit(self.pitch_limit * self.rp_limit_overhead)
        self.pid_vy.set_output_limit(self.roll_limit * self.rp_limit_overhead)
        self.pid_vz.set_output_limit(32767.5 / self.thrust_scale)  # UINT16_MAX/2 / scale

        # Transform current velocity to body frame
        state_body_vx, state_body_vy = self._world_to_body_xy(
            velocity[:, 0], velocity[:, 1], yaw
        )

        # Velocity PID for roll and pitch
        pitch_desired = self.pid_vx.update_with_setpoint(setpoint_velocity[:, 0], state_body_vx, False)
        roll_desired = -self.pid_vy.update_with_setpoint(setpoint_velocity[:, 1], state_body_vy, False)

        # Constrain roll/pitch
        roll_desired = torch.clamp(roll_desired, -self.roll_limit, self.roll_limit)
        pitch_desired = torch.clamp(pitch_desired, -self.pitch_limit, self.pitch_limit)

        # Thrust from Z velocity
        thrust_raw = self.pid_vz.update_with_setpoint(setpoint_velocity[:, 2], velocity[:, 2], False)

        # Scale thrust and add feedforward base
        thrust = thrust_raw * self.thrust_scale + self.thrust_base

        # Apply minimum thrust
        thrust = torch.maximum(thrust, torch.full_like(thrust, self.thrust_min))

        # Saturate to uint16
        thrust = torch.clamp(thrust, 0.0, 65535.0)

        return roll_desired, pitch_desired, thrust

    def update(
        self,
        position: torch.Tensor,
        velocity: torch.Tensor,
        yaw: torch.Tensor,
        setpoint_position: torch.Tensor,
        setpoint_velocity: torch.Tensor,
        mode_position: torch.Tensor,
        velocity_body: bool = False,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Full cascaded position control update.

        Args:
            position: Current position [num_envs, 3] (x, y, z) in meters
            velocity: Current velocity [num_envs, 3] (vx, vy, vz) in m/s
            yaw: Current yaw angle [num_envs] in degrees
            setpoint_position: Desired position [num_envs, 3] in meters
            setpoint_velocity: Desired velocity [num_envs, 3] in m/s (feedforward)
            mode_position: Position control mode [num_envs, 3] (True = position, False = velocity)
            velocity_body: If True, setpoint_velocity is body-frame

        Returns:
            Tuple of (roll_desired, pitch_desired, thrust)
        """
        # Position -> Velocity
        vel_x, vel_y, vel_z = self.position_control(
            position, velocity, yaw,
            setpoint_position, setpoint_velocity,
            mode_position[:, 0], mode_position[:, 1], mode_position[:, 2],
            velocity_body=velocity_body,
        )

        vel_setpoint = torch.stack([vel_x, vel_y, vel_z], dim=1)

        # Velocity -> Attitude + Thrust
        return self.velocity_control(velocity, yaw, vel_setpoint)
