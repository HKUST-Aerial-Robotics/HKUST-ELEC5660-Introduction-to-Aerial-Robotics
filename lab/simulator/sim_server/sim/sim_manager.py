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
        self.camera_left, self.camera_right = self._setup_cameras()

        # Controller
        self.controller = CrazyflieController(num_envs=1, device=self.device)

        # Cache body ids after PhysX view is initialized.
        self._body_ids = None

        # ROS2
        fx, fy, cx, cy = cfg.camera.intrinsics()
        self.ros = RosInterface(
            control_topic=cfg.control_topic,
            velocity_topic=cfg.velocity_topic,
            position_topic=cfg.position_topic,
            odom_topic=cfg.odom_topic,
            left_topic=cfg.camera.left_topic,
            right_topic=cfg.camera.right_topic,
            reset_topic=cfg.reset_topic,
            left_info_topic=cfg.camera.left_info_topic,
            right_info_topic=cfg.camera.right_info_topic,
            cam_width=cfg.camera.width,
            cam_height=cfg.camera.height,
            cam_fx=fx,
            cam_fy=fy,
            cam_cx=cx,
            cam_cy=cy,
            cam_baseline=cfg.camera.baseline,
            left_frame_id=cfg.camera.left_frame_id,
            right_frame_id=cfg.camera.right_frame_id,
        )

        # Preallocate command tensors to avoid per-step allocations.
        self._cmd_roll = torch.zeros(1, device=self.device)
        self._cmd_pitch = torch.zeros(1, device=self.device)
        self._cmd_yaw = torch.zeros(1, device=self.device)
        self._cmd_thrust = torch.zeros(1, device=self.device)
        self._cmd_vx = torch.zeros(1, device=self.device)
        self._cmd_vy = torch.zeros(1, device=self.device)
        self._cmd_vz = torch.zeros(1, device=self.device)
        self._cmd_yaw_rate = torch.zeros(1, device=self.device)
        self._cmd_px = torch.zeros(1, device=self.device)
        self._cmd_py = torch.zeros(1, device=self.device)
        self._cmd_pz = torch.zeros(1, device=self.device)

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
        body_prim = f"{self.cfg.robot_prim_path}/{self.cfg.robot_body_prim}"

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
            prim_path=f"{body_prim}/StereoLeft",
            width=cam_cfg.width,
            height=cam_cfg.height,
            update_period=update_period,
            data_types=["rgb"],
            offset=left_offset,
            spawn=left_pinhole_cfg,
        )
        right_cfg = CameraCfg(
            prim_path=f"{body_prim}/StereoRight",
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
        self._cache_body_ids()

        state = self._get_control_state()
        self.controller.reset(
            state={
                "position": state["position"],
                "attitude": state["attitude"],
            }
        )
        self._set_hover_setpoint(state["attitude"][:, 2])

    def _get_control_state(self) -> dict[str, torch.Tensor]:
        root_state = self.robot.data.root_state_w

        position = root_state[:, :3]
        velocity = root_state[:, 7:10]
        orientation = root_state[:, 3:7]

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

    def _get_odom_state(self) -> dict[str, torch.Tensor]:
        root_state = self.robot.data.root_state_w
        return {
            "position": root_state[:, :3],
            "orientation": root_state[:, 3:7],
            "velocity": root_state[:, 7:10],
            "angular_velocity_world": root_state[:, 10:13],
        }

    def _apply_control(self, state: dict[str, torch.Tensor]) -> None:
        force, torque = self.controller.compute(state)
        self._set_robot_forces(force, torque)

    def _set_robot_forces(self, forces: torch.Tensor, torques: torch.Tensor) -> None:
        forces_reshaped = forces.unsqueeze(1)
        torques_reshaped = torques.unsqueeze(1)
        if self._body_ids is None:
            self._cache_body_ids()
        self.robot.set_external_force_and_torque(
            forces=forces_reshaped,
            torques=torques_reshaped,
            body_ids=self._body_ids,
            is_global=False,
        )
        self.robot.write_data_to_sim()

    def _cache_body_ids(self) -> None:
        body_name = self.cfg.robot_body_prim.split("/")[-1]
        try:
            self._body_ids = self.robot.find_bodies(body_name)[0]
        except Exception:
            self._body_ids = self.robot.find_bodies("body")[0]


    def _update_setpoint_from_ros(self) -> bool:
        cmd = self.ros.get_latest_command(self.cfg.command_timeout_s)
        if cmd is None:
            return False

        if cmd.mode == ControlMode.ATTITUDE and cmd.attitude is not None:
            self._cmd_roll[0] = cmd.attitude.roll_deg
            self._cmd_pitch[0] = cmd.attitude.pitch_deg
            self._cmd_yaw[0] = cmd.attitude.yaw_deg
            self._cmd_thrust[0] = cmd.attitude.thrust

            self.controller.set_attitude_setpoint(
                roll_deg=self._cmd_roll,
                pitch_deg=self._cmd_pitch,
                yaw_deg=self._cmd_yaw,
                thrust=self._cmd_thrust,
            )
            return True

        if cmd.mode == ControlMode.VELOCITY and cmd.velocity is not None:
            vel = cmd.velocity
            self._cmd_vx[0] = vel.vx
            self._cmd_vy[0] = vel.vy
            self._cmd_vz[0] = vel.vz
            self._cmd_yaw_rate[0] = vel.yaw_rate_dps
            self.controller.set_velocity_setpoint(
                vx=self._cmd_vx,
                vy=self._cmd_vy,
                vz=self._cmd_vz,
                yaw_rate=self._cmd_yaw_rate,
                velocity_body=vel.body_frame,
            )
            return True

        if cmd.mode == ControlMode.POSITION and cmd.position is not None:
            pos = cmd.position
            self._cmd_px[0] = pos.x
            self._cmd_py[0] = pos.y
            self._cmd_pz[0] = pos.z
            yaw_t = None
            if pos.yaw_deg is not None:
                self._cmd_yaw[0] = pos.yaw_deg
                yaw_t = self._cmd_yaw

            self.controller.set_position_setpoint(
                x=self._cmd_px,
                y=self._cmd_py,
                z=self._cmd_pz,
                yaw=yaw_t,
            )
            return True

        return False

    def _set_hover_setpoint(self, yaw: torch.Tensor) -> None:
        target = self.cfg.robot_init_pos
        self._cmd_px[0] = target[0]
        self._cmd_py[0] = target[1]
        self._cmd_pz[0] = target[2]
        self.controller.set_position_setpoint(
            x=self._cmd_px,
            y=self._cmd_py,
            z=self._cmd_pz,
            yaw=yaw,
        )

    def _publish_odom(self, sim_time_s: float, state: dict[str, torch.Tensor]) -> None:
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
        self.ros.publish_camera_info(sim_time_s=sim_time_s, is_left=True)
        self.ros.publish_image(
            sim_time_s=sim_time_s,
            frame_id=self.cfg.camera.right_frame_id,
            image=right[0].detach(),
            is_left=False,
        )
        self.ros.publish_camera_info(sim_time_s=sim_time_s, is_left=False)

    def run(self) -> None:
        physics_dt = self.sim.get_physics_dt()
        render_interval = max(1, int(round(self.cfg.render_dt / physics_dt)))
        camera_dt = render_interval * physics_dt

        self._running = True
        sim_time_s = 0.0
        step_count = 0

        with torch.no_grad():
            while self.simulation_app.is_running() and self._running:
                # ROS2 I/O
                self.ros.spin_once()
                if self.ros.consume_reset_request():
                    self._reset()
                    self.ros.clear_commands()
                    continue

                control_state = self._get_control_state()
                has_cmd = self._update_setpoint_from_ros()
                if not has_cmd:
                    self._set_hover_setpoint(control_state["attitude"][:, 2])

                # Control
                self._apply_control(control_state)

                # Step sim
                render = (step_count % render_interval) == 0
                self.sim.step(render=render)
                self.robot.update(physics_dt)

                sim_time_s += physics_dt

                # Odom at physics rate (no expensive conversions)
                odom_state = self._get_odom_state()
                self._publish_odom(sim_time_s, odom_state)

                # Camera at render rate
                if render:
                    self.camera_left.update(dt=camera_dt)
                    self.camera_right.update(dt=camera_dt)
                    self._publish_stereo(sim_time_s)

                step_count += 1

        self.ros.shutdown()

    def stop(self) -> None:
        self._running = False
