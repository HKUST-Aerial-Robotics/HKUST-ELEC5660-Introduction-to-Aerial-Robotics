"""Minimal simulator configuration for ELEC5660 single-drone sim."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class CameraConfig:
    """Stereo camera configuration."""

    # RealSense D435i stereo defaults (IR pair)
    width: int = 640
    height: int = 480
    fps: int = 25

    # Pinhole intrinsics (Isaac Sim units: mm for focal/aperture)
    # These are chosen to match D435i HFOV/VFOV (~87°/58°) via aperture ratios.
    focal_length: float = 1.0
    horizontal_aperture: float = 1.8979291334297592
    vertical_aperture: float = 1.108618102905538
    clipping_range: tuple[float, float] = (0.05, 100.0)

    # Mounting relative to robot frame (ROS convention)
    forward_offset: float = 0.1
    up_offset: float = 0.02
    baseline: float = 0.05

    rot_ros: tuple[float, float, float, float] = (0.5, -0.5, 0.5, -0.5)

    left_frame_id: str = "world"
    right_frame_id: str = "world"
    left_topic: str = "/stereo/left/image_raw/compressed"
    right_topic: str = "/stereo/right/image_raw/compressed"
    left_info_topic: str = "/stereo/left/camera_info"
    right_info_topic: str = "/stereo/right/camera_info"

    def intrinsics(self) -> tuple[float, float, float, float]:
        """Return (fx, fy, cx, cy) in pixel units."""
        fx = self.focal_length / self.horizontal_aperture * self.width
        fy = self.focal_length / self.vertical_aperture * self.height
        cx = (self.width - 1) * 0.5
        cy = (self.height - 1) * 0.5
        return fx, fy, cx, cy


@dataclass
class SimCfg:
    """Top-level sim configuration."""

    # Timing
    physics_dt: float = 0.01  # 100 Hz
    render_dt: float = 0.04  # 25 Hz

    # Scene
    scene_usd: str | None = "/workspace/isaaclab/simulator/sim_server/assets/lab_env/lab_env.usd"
    # scene_usd: str | None = None
    scene_prim_path: str = "/World/Environment"

    # Robot
    robot_prim_path: str = "/World/OmniNxt"
    robot_body_prim: str = "body/body"
    robot_init_pos: tuple[float, float, float] = (0.0, 0.0, 0.5)

    # ROS2
    control_topic: str = "/sim/setpoint_raw/attitude"
    velocity_topic: str = "/sim/setpoint_velocity/cmd_vel"
    position_topic: str = "/sim/setpoint_position/local"
    reset_topic: str = "/sim/reset"
    odom_topic: str = "/sim/odom"
    odom_frame_id: str = "world"
    base_frame_id: str = "base_link"
    command_timeout_s: float = 0.5

    camera: CameraConfig = field(default_factory=CameraConfig)


LOG_LEVEL = "INFO"
LOG_FORMAT = "[%(levelname)s] %(message)s"
