#!/usr/bin/env python3
"""ROS1 adaptor for ELEC5660 simulator.

Bridges simulator odom to px4ctrl expectations, mocks MAVROS state/services,
translates AttitudeTarget to Float32MultiArray for ROS2 sim input, and maps
Xbox /joy to mavros RC channels.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, ExtendedState, RCIn, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse
from mavros_msgs.srv import CommandLong, CommandLongResponse
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Joy
from std_msgs.msg import Empty, Float32MultiArray


def quat_to_rot(q: Quaternion) -> List[List[float]]:
    """Return 3x3 rotation matrix for body->world from quaternion."""
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    xx = x * x
    yy = y * y
    zz = z * z
    ww = w * w

    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz],
    ]


def rot_transpose_vec(rot: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Apply R^T to vector v."""
    vx, vy, vz = v
    return (
        rot[0][0] * vx + rot[1][0] * vy + rot[2][0] * vz,
        rot[0][1] * vx + rot[1][1] * vy + rot[2][1] * vz,
        rot[0][2] * vx + rot[1][2] * vy + rot[2][2] * vz,
    )

def _quat_slerp(q0: Quaternion, q1: Quaternion, t: float) -> Quaternion:
    w0, x0, y0, z0 = q0.w, q0.x, q0.y, q0.z
    w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
    dot = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    if dot < 0.0:
        w1, x1, y1, z1 = -w1, -x1, -y1, -z1
        dot = -dot
    if dot > 0.9995:
        w = w0 + t * (w1 - w0)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
    else:
        theta_0 = math.acos(dot)
        sin_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_t = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_t / sin_0
        s1 = sin_t / sin_0
        w = s0 * w0 + s1 * w1
        x = s0 * x0 + s1 * x1
        y = s0 * y0 + s1 * y1
        z = s0 * z0 + s1 * z1
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 1e-9:
        w /= norm
        x /= norm
        y /= norm
        z /= norm
    return Quaternion(x=x, y=y, z=z, w=w)


class SimAdaptor:
    def __init__(self) -> None:
        self.sim_odom_topic = rospy.get_param("~sim_odom_topic", "/sim/odom")
        self.ekf_odom_topic = rospy.get_param("~ekf_odom_topic", "/ekf/ekf_odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.extended_state_topic = rospy.get_param("~extended_state_topic", "/mavros/extended_state")
        self.battery_topic = rospy.get_param("~battery_topic", "/mavros/battery")
        self.rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        self.attitude_in_topic = rospy.get_param("~attitude_in_topic", "/mavros/setpoint_raw/attitude")
        self.attitude_out_topic = rospy.get_param("~attitude_out_topic", "/sim/setpoint_raw/attitude")
        self.max_thrust = float(rospy.get_param("~max_thrust", 16.7))
        self.reset_topic = rospy.get_param("~reset_topic", "/sim/reset")

        self.gravity = float(rospy.get_param("~gravity", 9.81))

        self.rc_rate_hz = float(rospy.get_param("~rc_rate_hz", 50.0))
        self.state_rate_hz = float(rospy.get_param("~state_rate_hz", 10.0))
        self.battery_rate_hz = float(rospy.get_param("~battery_rate_hz", 1.0))

        self.rc_channel_count = int(rospy.get_param("~rc_channel_count", 8))
        self.mode_channel = int(rospy.get_param("~mode_channel", 6))
        self.command_channel = int(rospy.get_param("~command_channel", 7))

        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.5))
        self.deadzone = float(rospy.get_param("~deadzone", 0.1))

        self.axis_left_x = int(rospy.get_param("~axis_left_x", 0))
        self.axis_left_y = int(rospy.get_param("~axis_left_y", 1))
        self.axis_right_x = int(rospy.get_param("~axis_right_x", 2))
        self.axis_right_y = int(rospy.get_param("~axis_right_y", 3))

        self.invert_left_x = bool(rospy.get_param("~invert_left_x", True))
        self.invert_left_y = bool(rospy.get_param("~invert_left_y", False))
        self.invert_right_x = bool(rospy.get_param("~invert_right_x", False))
        self.invert_right_y = bool(rospy.get_param("~invert_right_y", True))

        self.button_mode = int(rospy.get_param("~button_mode", 7))
        self.button_command = int(rospy.get_param("~button_command", 6))
        self.button_reset = int(rospy.get_param("~button_reset", -1))

        self.default_mode_on = bool(rospy.get_param("~default_mode_on", False))
        self.default_command_on = bool(rospy.get_param("~default_command_on", False))

        self._connected = False
        self._armed = False
        self._mode = "MANUAL"

        self._last_odom: Optional[Odometry] = None
        self._last_vel: Optional[Tuple[float, float, float]] = None
        self._last_odom_time: Optional[rospy.Time] = None

        self._last_joy: Optional[Joy] = None
        self._last_joy_time: rospy.Time = rospy.Time(0)
        self._prev_buttons: Dict[int, bool] = {}

        self._mode_on = self.default_mode_on
        self._command_on = self.default_command_on

        self.ekf_pub = rospy.Publisher(self.ekf_odom_topic, Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, State, queue_size=10)
        self.ext_state_pub = rospy.Publisher(self.extended_state_topic, ExtendedState, queue_size=10)
        self.battery_pub = rospy.Publisher(self.battery_topic, BatteryState, queue_size=10)
        self.rc_pub = rospy.Publisher(self.rc_topic, RCIn, queue_size=10)
        self.att_pub = rospy.Publisher(self.attitude_out_topic, Float32MultiArray, queue_size=10)
        self.reset_pub = rospy.Publisher(self.reset_topic, Empty, queue_size=1, latch=True)

        rospy.Subscriber(self.sim_odom_topic, Odometry, self._on_sim_odom, queue_size=50)
        rospy.Subscriber(self.attitude_in_topic, AttitudeTarget, self._on_attitude_target, queue_size=50)
        rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=10)

        rospy.Service("/mavros/set_mode", SetMode, self._on_set_mode)
        rospy.Service("/mavros/cmd/arming", CommandBool, self._on_arm)
        rospy.Service("/mavros/cmd/command", CommandLong, self._on_command)

        if self.rc_rate_hz > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.rc_rate_hz), self._publish_rc)
        if self.state_rate_hz > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.state_rate_hz), self._publish_state)
            rospy.Timer(rospy.Duration(1.0 / self.state_rate_hz), self._publish_extended_state)
        if self.battery_rate_hz > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.battery_rate_hz), self._publish_battery)

    def _apply_deadzone(self, val: float) -> float:
        if abs(val) < self.deadzone:
            return 0.0
        sign = 1.0 if val > 0.0 else -1.0
        return sign * (abs(val) - self.deadzone) / (1.0 - self.deadzone)

    @staticmethod
    def _pwm_from_axis(val: float) -> int:
        pwm = 1500.0 + 500.0 * val
        if pwm < 1000.0:
            pwm = 1000.0
        elif pwm > 2000.0:
            pwm = 2000.0
        return int(pwm)

    def _axis(self, idx: int, invert: bool) -> float:
        if self._last_joy is None or idx < 0 or idx >= len(self._last_joy.axes):
            return 0.0
        val = float(self._last_joy.axes[idx])
        if invert:
            val = -val
        return self._apply_deadzone(val)

    def _button(self, idx: int) -> bool:
        if self._last_joy is None or idx < 0 or idx >= len(self._last_joy.buttons):
            return False
        return bool(self._last_joy.buttons[idx])

    def _button_edge(self, idx: int) -> bool:
        pressed = self._button(idx)
        was = self._prev_buttons.get(idx, False)
        self._prev_buttons[idx] = pressed
        return pressed and not was

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg
        self._last_joy_time = rospy.Time.now()

        if self.button_mode >= 0 and self._button_edge(self.button_mode):
            self._mode_on = not self._mode_on
            if not self._mode_on:
                self._command_on = False

        if self.button_command >= 0 and self._button_edge(self.button_command):
            self._command_on = not self._command_on

        if self.button_reset >= 0 and self._button_edge(self.button_reset):
            self.reset_pub.publish(Empty())

    def _on_set_mode(self, req) -> SetModeResponse:
        if req.custom_mode:
            self._mode = req.custom_mode
        elif req.base_mode:
            self._mode = "OFFBOARD" if req.base_mode else self._mode
        return SetModeResponse(mode_sent=True)

    def _on_arm(self, req) -> CommandBoolResponse:
        self._armed = bool(req.value)
        return CommandBoolResponse(success=True, result=0)

    def _on_command(self, _req) -> CommandLongResponse:
        return CommandLongResponse(success=True, result=0)

    def _on_attitude_target(self, msg: AttitudeTarget) -> None:
        thrust = float(msg.thrust)
        if thrust < 0.0:
            thrust = 0.0
        elif thrust > 1.0:
            thrust = 1.0
        thrust *= self.max_thrust
        out = Float32MultiArray()
        out.data = [
            float(msg.type_mask),
            float(msg.orientation.w),
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.body_rate.x),
            float(msg.body_rate.y),
            float(msg.body_rate.z),
            thrust,
        ]
        self.att_pub.publish(out)

    def _on_sim_odom(self, msg: Odometry) -> None:
        self._connected = True
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        vel = msg.twist.twist.linear
        vel_tuple = (float(vel.x), float(vel.y), float(vel.z))

        if self._last_odom is None or self._last_vel is None or self._last_odom_time is None:
            self._publish_odom_and_imu(msg, (0.0, 0.0, 0.0))
            self._last_odom = msg
            self._last_vel = vel_tuple
            self._last_odom_time = stamp
            return

        dt = (stamp - self._last_odom_time).to_sec()
        if dt <= 1e-3 or dt > 1.0:
            self._publish_odom_and_imu(msg, (0.0, 0.0, 0.0))
            self._last_odom = msg
            self._last_vel = vel_tuple
            self._last_odom_time = stamp
            return

        a_world = (
            (vel_tuple[0] - self._last_vel[0]) / dt,
            (vel_tuple[1] - self._last_vel[1]) / dt,
            (vel_tuple[2] - self._last_vel[2]) / dt,
        )

        mid = Odometry()
        mid.header = msg.header
        mid.header.stamp = self._last_odom_time + rospy.Duration.from_sec(0.5 * dt)
        mid.child_frame_id = msg.child_frame_id

        p0 = self._last_odom.pose.pose.position
        p1 = msg.pose.pose.position
        mid.pose.pose.position.x = 0.5 * (p0.x + p1.x)
        mid.pose.pose.position.y = 0.5 * (p0.y + p1.y)
        mid.pose.pose.position.z = 0.5 * (p0.z + p1.z)
        mid.pose.pose.orientation = _quat_slerp(self._last_odom.pose.pose.orientation, msg.pose.pose.orientation, 0.5)

        w0 = self._last_odom.twist.twist.angular
        w1 = msg.twist.twist.angular
        mid.twist.twist.linear.x = 0.5 * (self._last_vel[0] + vel_tuple[0])
        mid.twist.twist.linear.y = 0.5 * (self._last_vel[1] + vel_tuple[1])
        mid.twist.twist.linear.z = 0.5 * (self._last_vel[2] + vel_tuple[2])
        mid.twist.twist.angular.x = 0.5 * (w0.x + w1.x)
        mid.twist.twist.angular.y = 0.5 * (w0.y + w1.y)
        mid.twist.twist.angular.z = 0.5 * (w0.z + w1.z)

        self._publish_odom_and_imu(mid, a_world)
        self._publish_odom_and_imu(msg, a_world)

        self._last_odom = msg
        self._last_vel = vel_tuple
        self._last_odom_time = stamp

    def _publish_odom_and_imu(self, msg: Odometry, a_world: Tuple[float, float, float]) -> None:
        self.ekf_pub.publish(msg)
        a_world = (a_world[0], a_world[1], a_world[2] + self.gravity)
        quat = msg.pose.pose.orientation
        rot = quat_to_rot(quat)
        a_body = rot_transpose_vec(rot, a_world)
        w_world = msg.twist.twist.angular
        w_body = rot_transpose_vec(rot, (float(w_world.x), float(w_world.y), float(w_world.z)))

        imu = Imu()
        imu.header = msg.header
        imu.orientation = quat
        imu.angular_velocity.x = w_body[0]
        imu.angular_velocity.y = w_body[1]
        imu.angular_velocity.z = w_body[2]
        imu.linear_acceleration.x = a_body[0]
        imu.linear_acceleration.y = a_body[1]
        imu.linear_acceleration.z = a_body[2]
        self.imu_pub.publish(imu)

    def _publish_state(self, _event: rospy.TimerEvent) -> None:
        state = State()
        if hasattr(state, "header"):
            state.header.stamp = rospy.Time.now()
        state.connected = self._connected
        state.armed = self._armed
        state.mode = self._mode
        if hasattr(state, "guided"):
            state.guided = self._mode.upper() == "OFFBOARD"
        if hasattr(state, "manual_input"):
            state.manual_input = not getattr(state, "guided", False)
        state.system_status = 3  # MAV_STATE_ACTIVE
        self.state_pub.publish(state)

    def _publish_extended_state(self, _event: rospy.TimerEvent) -> None:
        ext = ExtendedState()
        if hasattr(ext, "header"):
            ext.header.stamp = rospy.Time.now()
        ext.vtol_state = ExtendedState.VTOL_STATE_UNDEFINED

        if not self._armed:
            ext.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        elif self._last_odom is None:
            ext.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        else:
            z = self._last_odom.pose.pose.position.z
            vz = self._last_odom.twist.twist.linear.z
            if z < 0.1 and abs(vz) < 0.1:
                ext.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
            else:
                ext.landed_state = ExtendedState.LANDED_STATE_IN_AIR

        self.ext_state_pub.publish(ext)

    def _publish_battery(self, _event: rospy.TimerEvent) -> None:
        msg = BatteryState()
        if hasattr(msg, "header"):
            msg.header.stamp = rospy.Time.now()
        msg.cell_voltage = [4.0, 4.0, 4.0, 4.0]
        msg.voltage = sum(msg.cell_voltage)
        msg.percentage = 1.0
        self.battery_pub.publish(msg)

    def _publish_rc(self, _event: rospy.TimerEvent) -> None:
        now = rospy.Time.now()
        if (now - self._last_joy_time).to_sec() > self.joy_timeout:
            self._last_joy = None

        roll = self._axis(self.axis_right_x, self.invert_right_x)
        pitch = self._axis(self.axis_right_y, self.invert_right_y)
        throttle = self._axis(self.axis_left_y, self.invert_left_y)
        yaw = self._axis(self.axis_left_x, self.invert_left_x)

        channels = [1500] * max(self.rc_channel_count, 8)
        channels[0] = self._pwm_from_axis(roll)
        channels[1] = self._pwm_from_axis(pitch)
        channels[2] = self._pwm_from_axis(throttle)
        channels[3] = self._pwm_from_axis(yaw)

        mode_idx = max(0, self.mode_channel - 1)
        cmd_idx = max(0, self.command_channel - 1)
        if mode_idx >= len(channels):
            channels.extend([1500] * (mode_idx + 1 - len(channels)))
        if cmd_idx >= len(channels):
            channels.extend([1500] * (cmd_idx + 1 - len(channels)))

        channels[mode_idx] = 2000 if self._mode_on else 1000
        channels[cmd_idx] = 2000 if self._command_on else 1000

        msg = RCIn()
        if hasattr(msg, "header"):
            msg.header.stamp = now
        msg.rssi = 255
        msg.channels = channels
        self.rc_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("sim_adaptor")
    SimAdaptor()
    rospy.spin()
