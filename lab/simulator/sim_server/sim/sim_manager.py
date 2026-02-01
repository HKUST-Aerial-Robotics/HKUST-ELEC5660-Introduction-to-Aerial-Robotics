"""Single-drone simulation manager for ELEC5660."""
from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.utils.math import euler_xyz_from_quat, matrix_from_quat
from pxr import UsdGeom

from assets.omninxt.omninxt import OMNINXT_CFG

from config import SimCfg
from controller.controller import CrazyflieController
from controller import config as ctrl_config
import isaacsim.core.utils.prims as prims_utils
from sim.ros_interface import ControlMode, RosInterface

if TYPE_CHECKING:
    from isaacsim import SimulationApp

logger = logging.getLogger(__name__)


class SimulationManager:
    """Minimal simulation loop for a single Crazyflie."""

    def __init__(self, simulation_app: SimulationApp, cfg: SimCfg):
        self.simulation_app = simulation_app
        self.cfg = cfg
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        self._running = False

        # Build simulation
        self.sim = self._setup_simulation()
        self.robot = self._setup_robot()
        self._apply_robot_physics_overrides()
        sim_utils.update_stage()
        self.camera_left, self.camera_right = self._setup_cameras()

        # Controller
        self.controller = CrazyflieController(num_envs=1, device=self.device)

        # ROS2
        self.ros = RosInterface(
            control_topic=cfg.control_topic,
            velocity_topic=cfg.velocity_topic,
            position_topic=cfg.position_topic,
            odom_topic=cfg.odom_topic,
            left_topic=cfg.camera.left_topic,
            right_topic=cfg.camera.right_topic,
            reset_topic=cfg.reset_topic,
        )

        self._reset()

    def _setup_simulation(self) -> SimulationContext:
        sim_cfg = SimulationCfg(dt=self.cfg.physics_dt, device=str(self.device))
        sim = SimulationContext(sim_cfg)
        sim.set_camera_view(eye=[1.2, 1.2, 1.0], target=[0.0, 0.0, 0.5])

        # Scene USD
        scene_usd = self.cfg.scene_usd
        if scene_usd is None:
            from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

            scene_usd = f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"
        env_cfg = sim_utils.UsdFileCfg(usd_path=scene_usd)
        env_cfg.func(self.cfg.scene_prim_path, env_cfg)

        # Light
        light_cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(1.0, 1.0, 1.0))
        light_cfg.func("/World/DomeLight", light_cfg)

        return sim

    def _setup_robot(self) -> Articulation:
        robot_cfg = OMNINXT_CFG.replace(prim_path=self.cfg.robot_prim_path)
        robot_cfg.init_state.pos = self.cfg.robot_init_pos
        robot_cfg.spawn.func(
            self.cfg.robot_prim_path,
            robot_cfg.spawn,
            translation=robot_cfg.init_state.pos,
            orientation=self.cfg.robot_spawn_rot,
        )
        return Articulation(robot_cfg)

    def _apply_robot_physics_overrides(self) -> None:
        """Override robot mass/inertia to match controller assumptions."""
        body_prim = f"{self.cfg.robot_prim_path}/{self.cfg.robot_body_prim}"
        try:
            prims_utils.set_prim_property(body_prim, "physics:mass", float(ctrl_config.CF_MASS))
            inertia = (
                float(ctrl_config.INERTIA_XX),
                float(ctrl_config.INERTIA_YY),
                float(ctrl_config.INERTIA_ZZ),
            )
            prims_utils.set_prim_property(body_prim, "physics:diagonalInertia", inertia)
        except Exception as exc:
            logger.warning("Failed to override robot mass/inertia: %s", exc)

    def _setup_cameras(self) -> tuple[Camera, Camera]:
        cam_cfg = self.cfg.camera
        update_period = 1.0 / float(cam_cfg.fps)

        left_offset = CameraCfg.OffsetCfg(
            pos=(cam_cfg.forward_offset, cam_cfg.baseline * 0.5, cam_cfg.up_offset),
            rot=cam_cfg.rot_ros,
            convention="ros",
        )
        right_offset = CameraCfg.OffsetCfg(
            pos=(cam_cfg.forward_offset, -cam_cfg.baseline * 0.5, cam_cfg.up_offset),
            rot=cam_cfg.rot_ros,
            convention="ros",
        )

        left_pinhole_cfg = sim_utils.PinholeCameraCfg(
            focal_length=cam_cfg.focal_length,
            horizontal_aperture=cam_cfg.horizontal_aperture,
            vertical_aperture=cam_cfg.vertical_aperture,
            clipping_range=cam_cfg.clipping_range,
        )
        right_pinhole_cfg = sim_utils.PinholeCameraCfg(
            focal_length=cam_cfg.focal_length,
            horizontal_aperture=cam_cfg.horizontal_aperture,
            vertical_aperture=cam_cfg.vertical_aperture,
            clipping_range=cam_cfg.clipping_range,
        )

        left_cfg = CameraCfg(
            prim_path=f"{self.cfg.robot_prim_path}/StereoLeft",
            width=cam_cfg.width,
            height=cam_cfg.height,
            update_period=update_period,
            data_types=["rgb"],
            offset=left_offset,
            spawn=left_pinhole_cfg,
        )
        right_cfg = CameraCfg(
            prim_path=f"{self.cfg.robot_prim_path}/StereoRight",
            width=cam_cfg.width,
            height=cam_cfg.height,
            update_period=update_period,
            data_types=["rgb"],
            offset=right_offset,
            spawn=right_pinhole_cfg,
        )

        return Camera(left_cfg), Camera(right_cfg)

    def _reset(self) -> None:
        self.sim.reset()
        self.robot.reset()
        self.robot.update(self.sim.get_physics_dt())

        state = self._get_robot_state()
        self.controller.reset(
            state={
                "position": state["position"],
                "attitude": state["attitude"],
            }
        )
        self._set_hover_setpoint()

    def _get_robot_state(self) -> dict[str, torch.Tensor]:
        root_state = self.robot.data.root_state_w

        position = root_state[:, :3].clone()
        velocity = root_state[:, 7:10].clone()
        orientation = root_state[:, 3:7].clone()

        roll, pitch, yaw = euler_xyz_from_quat(orientation)
        euler_rad = torch.stack([roll, pitch, yaw], dim=-1)
        euler_rad = torch.remainder(euler_rad + math.pi, 2 * math.pi) - math.pi
        attitude = torch.rad2deg(euler_rad)

        omega_world = root_state[:, 10:13]
        rot_mat = matrix_from_quat(orientation)
        omega_body = torch.einsum("nij,nj->ni", rot_mat.transpose(-1, -2), omega_world)
        angular_velocity_deg = torch.rad2deg(omega_body)

        return {
            "position": position,
            "velocity": velocity,
            "orientation": orientation,
            "angular_velocity": angular_velocity_deg,
            "attitude": attitude,
            "angular_velocity_world": omega_world,
        }

    def _apply_control(self) -> None:
        state = self._get_robot_state()
        ctrl_state = {
            "position": state["position"],
            "velocity": state["velocity"],
            "attitude": state["attitude"],
            "angular_velocity": state["angular_velocity"],
        }
        force, torque = self.controller.compute(ctrl_state)
        self._set_robot_forces(force, torque)

    def _set_robot_forces(self, forces: torch.Tensor, torques: torch.Tensor) -> None:
        forces_reshaped = forces.unsqueeze(1)
        torques_reshaped = torques.unsqueeze(1)
        body_ids = self.robot.find_bodies("body")[0]
        # Use the permanent wrench composer to avoid deprecated API
        self.robot.permanent_wrench_composer.set_forces_and_torques(
            forces=forces_reshaped,
            torques=torques_reshaped,
            body_ids=body_ids,
        )
        self.robot.write_data_to_sim()

    def _update_setpoint_from_ros(self) -> bool:
        cmd = self.ros.get_latest_command(self.cfg.command_timeout_s)
        if cmd is None:
            return False

        if cmd.mode == ControlMode.ATTITUDE and cmd.attitude is not None:
            roll_norm = max(-1.0, min(1.0, cmd.attitude.roll_deg / 30.0))
            pitch_norm = max(-1.0, min(1.0, cmd.attitude.pitch_deg / 30.0))
            yaw_rate_norm = max(-1.0, min(1.0, cmd.attitude.yaw_rate_dps / 120.0))
            thrust_norm = max(0.0, min(1.0, cmd.attitude.thrust))

            roll_t = torch.tensor([roll_norm], device=self.device)
            pitch_t = torch.tensor([pitch_norm], device=self.device)
            yaw_rate_t = torch.tensor([yaw_rate_norm], device=self.device)
            thrust_t = torch.tensor([thrust_norm], device=self.device)

            self.controller.set_attitude_setpoint(
                roll=roll_t,
                pitch=pitch_t,
                yaw_rate=yaw_rate_t,
                thrust=thrust_t,
            )
            return True

        if cmd.mode == ControlMode.VELOCITY and cmd.velocity is not None:
            vel = cmd.velocity
            vx_t = torch.tensor([vel.vx], device=self.device)
            vy_t = torch.tensor([vel.vy], device=self.device)
            vz_t = torch.tensor([vel.vz], device=self.device)
            yaw_rate_t = torch.tensor([vel.yaw_rate_dps], device=self.device)
            self.controller.set_velocity_setpoint(
                vx=vx_t,
                vy=vy_t,
                vz=vz_t,
                yaw_rate=yaw_rate_t,
                velocity_body=vel.body_frame,
            )
            return True

        if cmd.mode == ControlMode.POSITION and cmd.position is not None:
            pos = cmd.position
            x_t = torch.tensor([pos.x], device=self.device)
            y_t = torch.tensor([pos.y], device=self.device)
            z_t = torch.tensor([pos.z], device=self.device)
            yaw_t = None
            if pos.yaw_deg is not None:
                yaw_t = torch.tensor([pos.yaw_deg], device=self.device)

            self.controller.set_position_setpoint(
                x=x_t,
                y=y_t,
                z=z_t,
                yaw=yaw_t,
            )
            return True

        return False

    def _set_hover_setpoint(self) -> None:
        state = self._get_robot_state()
        yaw = state["attitude"][:, 2]
        target = self.cfg.robot_init_pos
        self.controller.set_position_setpoint(
            x=torch.tensor([target[0]], device=self.device),
            y=torch.tensor([target[1]], device=self.device),
            z=torch.tensor([target[2]], device=self.device),
            yaw=yaw,
        )

    def _publish_odom(self, sim_time_s: float) -> None:
        state = self._get_robot_state()
        pos = state["position"][0].detach()
        quat = state["orientation"][0].detach()
        vel = state["velocity"][0].detach()
        ang_vel = state["angular_velocity_world"][0].detach()

        self.ros.publish_odom(
            sim_time_s=sim_time_s,
            frame_id=self.cfg.odom_frame_id,
            child_frame_id=self.cfg.base_frame_id,
            position=pos,
            orientation_wxyz=quat,
            linear_vel=vel,
            angular_vel=ang_vel,
        )

    def _publish_stereo(self, sim_time_s: float) -> None:
        left = self.camera_left.data.output.get("rgb")
        right = self.camera_right.data.output.get("rgb")
        if left is None or right is None:
            return

        self.ros.publish_image(
            sim_time_s=sim_time_s,
            frame_id=self.cfg.camera.left_frame_id,
            image=left[0].detach(),
            is_left=True,
        )
        self.ros.publish_image(
            sim_time_s=sim_time_s,
            frame_id=self.cfg.camera.right_frame_id,
            image=right[0].detach(),
            is_left=False,
        )

    def run(self) -> None:
        physics_dt = self.sim.get_physics_dt()
        render_interval = max(1, int(round(self.cfg.render_dt / physics_dt)))
        camera_dt = render_interval * physics_dt

        self._running = True
        sim_time_s = 0.0
        step_count = 0

        while self.simulation_app.is_running() and self._running:
            # ROS2 I/O
            self.ros.spin_once()
            if self.ros.consume_reset_request():
                self._reset()
                self.ros.clear_commands()
                continue
            has_cmd = self._update_setpoint_from_ros()
            if not has_cmd:
                self._set_hover_setpoint()

            # Control
            self._apply_control()

            # Step sim
            render = (step_count % render_interval) == 0
            self.sim.step(render=render)
            self.robot.update(physics_dt)

            sim_time_s += physics_dt

            # Odom at physics rate
            self._publish_odom(sim_time_s)

            # Camera at render rate
            if render:
                self.camera_left.update(dt=camera_dt)
                self.camera_right.update(dt=camera_dt)
                self._publish_stereo(sim_time_s)
            step_count += 1

        self.ros.shutdown()

    def stop(self) -> None:
        self._running = False
