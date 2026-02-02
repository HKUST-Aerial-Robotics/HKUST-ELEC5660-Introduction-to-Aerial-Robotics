"""ROS2 interface for control input and sensor output."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Empty, Float32MultiArray
import torch

from isaaclab.utils.math import euler_xyz_from_quat

@dataclass
class AttitudeTargetLite:
    """Minimal AttitudeTarget-compatible payload (no ROS-specific dependency)."""

    type_mask: int
    orientation: Quaternion
    body_rate: Vector3
    thrust: float

    IGNORE_ROLL_RATE = 1
    IGNORE_PITCH_RATE = 2
    IGNORE_YAW_RATE = 4
    IGNORE_THRUST = 64
    IGNORE_ATTITUDE = 128


@dataclass
class AttitudeCommand:
    """Parsed attitude command in degrees and thrust in Newtons."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    thrust: float


@dataclass
class VelocityCommand:
    """Velocity command in m/s and deg/s."""

    vx: float
    vy: float
    vz: float
    yaw_rate_dps: float
    body_frame: bool = True


@dataclass
class PositionCommand:
    """Position command in meters and yaw in degrees."""

    x: float
    y: float
    z: float
    yaw_deg: Optional[float]


class ControlMode(str, Enum):
    ATTITUDE = "attitude"
    VELOCITY = "velocity"
    POSITION = "position"


@dataclass
class ControlCommand:
    mode: ControlMode
    attitude: Optional[AttitudeCommand] = None
    velocity: Optional[VelocityCommand] = None
    position: Optional[PositionCommand] = None


def quat_to_euler_xyz(q: Quaternion) -> tuple[float, float, float]:
    """Convert quaternion (x,y,z,w) to roll, pitch, yaw in radians using IsaacLab math."""
    quat = torch.tensor([[q.w, q.x, q.y, q.z]], dtype=torch.float32)
    roll, pitch, yaw = euler_xyz_from_quat(quat)
    return float(roll.item()), float(pitch.item()), float(yaw.item())


def to_ros_time(sim_time_s: float) -> Time:
    sec = int(sim_time_s)
    nanosec = int((sim_time_s - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


class RosInterface:
    """ROS2 I/O for simulator."""

    def __init__(
        self,
        *,
        control_topic: str,
        velocity_topic: str,
        position_topic: str,
        odom_topic: str,
        left_topic: str,
        right_topic: str,
        reset_topic: str,
        left_info_topic: str,
        right_info_topic: str,
        cam_width: int,
        cam_height: int,
        cam_fx: float,
        cam_fy: float,
        cam_cx: float,
        cam_cy: float,
        cam_baseline: float,
        left_frame_id: str,
        right_frame_id: str,
    ):
        rclpy.init(args=None)
        self.node = rclpy.create_node("elec5660_sim")
        self._warned_no_cv2 = False

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self._att_sub = self.node.create_subscription(
            Float32MultiArray, control_topic, self._on_attitude_target, qos
        )
        self._vel_sub = self.node.create_subscription(
            TwistStamped, velocity_topic, self._on_velocity_cmd, qos
        )
        self._pos_sub = self.node.create_subscription(
            PoseStamped, position_topic, self._on_position_cmd, qos
        )
        self._reset_sub = self.node.create_subscription(Empty, reset_topic, self._on_reset, qos)
        self._odom_pub = self.node.create_publisher(Odometry, odom_topic, qos)
        self._left_pub = self.node.create_publisher(CompressedImage, left_topic, qos)
        self._right_pub = self.node.create_publisher(CompressedImage, right_topic, qos)
        self._left_info_pub = self.node.create_publisher(CameraInfo, left_info_topic, qos)
        self._right_info_pub = self.node.create_publisher(CameraInfo, right_info_topic, qos)

        self._last_msg: Optional[AttitudeTargetLite] = None
        self._last_msg_time: float = 0.0
        self._last_velocity: Optional[VelocityCommand] = None
        self._last_velocity_time: float = 0.0
        self._last_position: Optional[PositionCommand] = None
        self._last_position_time: float = 0.0
        self._reset_requested: bool = False

        self._cam_width = cam_width
        self._cam_height = cam_height
        self._cam_fx = cam_fx
        self._cam_fy = cam_fy
        self._cam_cx = cam_cx
        self._cam_cy = cam_cy
        self._cam_baseline = cam_baseline
        self._left_frame_id = left_frame_id
        self._right_frame_id = right_frame_id

    def _on_attitude_target(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        # Expected layout: [type_mask, q_w, q_x, q_y, q_z, wx, wy, wz, thrust]
        if len(data) < 9:
            return

        type_mask = int(data[0])
        orientation = Quaternion(x=float(data[2]), y=float(data[3]), z=float(data[4]), w=float(data[1]))
        body_rate = Vector3(x=float(data[5]), y=float(data[6]), z=float(data[7]))
        thrust = float(data[8])

        self._last_msg = AttitudeTargetLite(
            type_mask=type_mask,
            orientation=orientation,
            body_rate=body_rate,
            thrust=thrust,
        )
        self._last_msg_time = time.monotonic()

    def _on_velocity_cmd(self, msg: TwistStamped) -> None:
        frame_id = (msg.header.frame_id or "").lower()
        body_frame = frame_id not in ("world", "odom", "map")

        yaw_rate_dps = float(torch.rad2deg(torch.tensor(msg.twist.angular.z)).item())

        self._last_velocity = VelocityCommand(
            vx=float(msg.twist.linear.x),
            vy=float(msg.twist.linear.y),
            vz=float(msg.twist.linear.z),
            yaw_rate_dps=yaw_rate_dps,
            body_frame=body_frame,
        )
        self._last_velocity_time = time.monotonic()

    def _on_position_cmd(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        yaw_deg: Optional[float] = None
        if norm > 1e-6:
            _roll, _pitch, yaw = quat_to_euler_xyz(q)
            yaw_deg = float(torch.rad2deg(torch.tensor(yaw)).item())

        self._last_position = PositionCommand(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            yaw_deg=yaw_deg,
        )
        self._last_position_time = time.monotonic()

    def _on_reset(self, _msg: Empty) -> None:
        self._reset_requested = True

    def spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def get_latest_command(self, timeout_s: float) -> Optional[ControlCommand]:
        now = time.monotonic()
        candidates: list[tuple[float, ControlCommand]] = []

        if self._last_msg is not None:
            if timeout_s <= 0.0 or (now - self._last_msg_time) <= timeout_s:
                msg = self._last_msg
                use_attitude = (msg.type_mask & AttitudeTargetLite.IGNORE_ATTITUDE) == 0
                use_thrust = (msg.type_mask & AttitudeTargetLite.IGNORE_THRUST) == 0

                if use_attitude:
                    roll, pitch, yaw = quat_to_euler_xyz(msg.orientation)
                    roll_deg = float(torch.rad2deg(torch.tensor(roll)).item())
                    pitch_deg = float(torch.rad2deg(torch.tensor(pitch)).item())
                    yaw_deg = float(torch.rad2deg(torch.tensor(yaw)).item())
                else:
                    roll_deg = 0.0
                    pitch_deg = 0.0
                    yaw_deg = 0.0

                thrust = float(msg.thrust) if use_thrust else 0.0

                att_cmd = AttitudeCommand(
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    thrust=thrust,
                )
                candidates.append(
                    (
                        self._last_msg_time,
                        ControlCommand(mode=ControlMode.ATTITUDE, attitude=att_cmd),
                    )
                )

        if self._last_velocity is not None:
            if timeout_s <= 0.0 or (now - self._last_velocity_time) <= timeout_s:
                candidates.append(
                    (
                        self._last_velocity_time,
                        ControlCommand(mode=ControlMode.VELOCITY, velocity=self._last_velocity),
                    )
                )

        if self._last_position is not None:
            if timeout_s <= 0.0 or (now - self._last_position_time) <= timeout_s:
                candidates.append(
                    (
                        self._last_position_time,
                        ControlCommand(mode=ControlMode.POSITION, position=self._last_position),
                    )
                )

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def consume_reset_request(self) -> bool:
        if not self._reset_requested:
            return False
        self._reset_requested = False
        return True

    def clear_commands(self) -> None:
        self._last_msg = None
        self._last_msg_time = 0.0
        self._last_velocity = None
        self._last_velocity_time = 0.0
        self._last_position = None
        self._last_position_time = 0.0

    def publish_odom(
        self,
        *,
        sim_time_s: float,
        frame_id: str,
        child_frame_id: str,
        position: torch.Tensor,
        orientation_wxyz: torch.Tensor,
        linear_vel: torch.Tensor,
        angular_vel: torch.Tensor,
    ) -> None:
        msg = Odometry()
        msg.header.stamp = to_ros_time(sim_time_s)
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id

        msg.pose.pose.position.x = float(position[0].item())
        msg.pose.pose.position.y = float(position[1].item())
        msg.pose.pose.position.z = float(position[2].item())

        # ROS uses x,y,z,w
        msg.pose.pose.orientation.x = float(orientation_wxyz[1].item())
        msg.pose.pose.orientation.y = float(orientation_wxyz[2].item())
        msg.pose.pose.orientation.z = float(orientation_wxyz[3].item())
        msg.pose.pose.orientation.w = float(orientation_wxyz[0].item())

        msg.twist.twist.linear.x = float(linear_vel[0].item())
        msg.twist.twist.linear.y = float(linear_vel[1].item())
        msg.twist.twist.linear.z = float(linear_vel[2].item())

        msg.twist.twist.angular.x = float(angular_vel[0].item())
        msg.twist.twist.angular.y = float(angular_vel[1].item())
        msg.twist.twist.angular.z = float(angular_vel[2].item())

        self._odom_pub.publish(msg)

    def publish_image(
        self,
        *,
        sim_time_s: float,
        frame_id: str,
        image: torch.Tensor,
        is_left: bool,
    ) -> None:
        if image.ndim != 3:
            return
        if image.shape[2] == 4:
            image = image[:, :, :3]

        if image.dtype != torch.uint8:
            img = image
            if torch.is_floating_point(img):
                max_val = float(img.max().item()) if img.numel() > 0 else 1.0
                if max_val <= 1.0:
                    img = img * 255.0
            image = torch.clamp(img, 0.0, 255.0).to(torch.uint8)

        height, width, _ = image.shape

        # Encode to JPEG using OpenCV (requires numpy via torch -> numpy conversion)
        try:
            import cv2
        except Exception:
            if not self._warned_no_cv2:
                self.node.get_logger().warning("OpenCV not available, cannot publish CompressedImage.")
                self._warned_no_cv2 = True
            return

        img_cpu = image.contiguous().cpu()
        img_np = img_cpu.numpy()
        bgr = img_np[:, :, ::-1]
        ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = to_ros_time(sim_time_s)
        msg.header.frame_id = frame_id
        msg.format = "jpeg"
        msg.data = enc.tobytes()

        if is_left:
            self._left_pub.publish(msg)
        else:
            self._right_pub.publish(msg)

    def publish_camera_info(self, *, sim_time_s: float, is_left: bool) -> None:
        msg = CameraInfo()
        msg.header.stamp = to_ros_time(sim_time_s)
        msg.header.frame_id = self._left_frame_id if is_left else self._right_frame_id
        msg.width = int(self._cam_width)
        msg.height = int(self._cam_height)

        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = float(self._cam_fx)
        fy = float(self._cam_fy)
        cx = float(self._cam_cx)
        cy = float(self._cam_cy)

        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        tx = 0.0 if is_left else -fx * float(self._cam_baseline)
        msg.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        if is_left:
            self._left_info_pub.publish(msg)
        else:
            self._right_info_pub.publish(msg)

    def shutdown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()
