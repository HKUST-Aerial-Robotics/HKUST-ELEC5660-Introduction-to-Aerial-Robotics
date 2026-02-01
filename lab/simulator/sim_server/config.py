"""Minimal simulator configuration for ELEC5660 single-drone sim."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class CameraConfig:
    """Stereo camera configuration."""

    width: int = 320
    height: int = 240
    fps: int = 25

    # Pinhole intrinsics (Isaac Sim units: mm for focal/aperture)
    focal_length: float = 24.0
    horizontal_aperture: float = 20.955
    vertical_aperture: float = 20.955
    clipping_range: tuple[float, float] = (0.05, 100.0)

    # Mounting relative to robot frame (ROS convention)
    forward_offset: float = 0.05
    up_offset: float = 0.0
    baseline: float = 0.04

    # Camera rotation: align camera +Z (ROS) with robot +X
    rot_ros: tuple[float, float, float, float] = (0.70710678, 0.0, -0.70710678, 0.0)

    left_frame_id: str = "stereo_left"
    right_frame_id: str = "stereo_right"
    left_topic: str = "/stereo/left/image_raw/compressed"
    right_topic: str = "/stereo/right/image_raw/compressed"


@dataclass
class SimCfg:
    """Top-level sim configuration."""

    # Timing
    physics_dt: float = 0.01  # 100 Hz
    render_dt: float = 0.04  # 25 Hz

    # Scene
    # scene_usd: str | None = "/workspace/isaaclab/simulator/sim_server/assets/lab_env/lab_env.usd"
    scene_usd: str | None = None
    scene_prim_path: str = "/World/Environment"

    # Robot
    robot_prim_path: str = "/World/OmniNxt"
    robot_body_prim: str = "body/body"
    # Spawn orientation (w, x, y, z). Set None to use USD orientation.
    robot_spawn_rot: tuple[float, float, float, float] | None = None
    robot_init_pos: tuple[float, float, float] = (0.0, 0.0, 0.5)

    # ROS2
    control_topic: str = "/mavros/setpoint_raw/attitude"
    velocity_topic: str = "/mavros/setpoint_velocity/cmd_vel"
    position_topic: str = "/mavros/setpoint_position/local"
    reset_topic: str = "/sim/reset"
    odom_topic: str = "/sim/odom"
    odom_frame_id: str = "map"
    base_frame_id: str = "base_link"
    command_timeout_s: float = 0.5

    camera: CameraConfig = field(default_factory=CameraConfig)


LOG_LEVEL = "INFO"
LOG_FORMAT = "[%(levelname)s] %(message)s"
