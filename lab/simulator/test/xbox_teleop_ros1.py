#!/usr/bin/env python3
"""ROS1 Xbox teleop node for ELEC5660 simulator.

Publishes simulator setpoints:
- /sim/setpoint_raw/attitude (Float32MultiArray)
- /sim/setpoint_velocity/cmd_vel (TwistStamped)
- /sim/setpoint_position/local (PoseStamped)

Input: /joy (sensor_msgs/Joy)
"""
import math
import os
import signal
import subprocess
from enum import IntEnum
from typing import Tuple

import rospy
import rosgraph

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, Float32MultiArray


CONTROL_RATE_HZ = 50
DEADZONE = 0.1

# Xbox Controller Axes (Linux/xpad)
AXIS_LEFT_X = 0   # Yaw
AXIS_LEFT_Y = 1   # Throttle/Alt (inverted)
AXIS_RIGHT_X = 2  # Roll/Strafe
AXIS_RIGHT_Y = 3  # Pitch/Forward (inverted)

# Xbox Controller Buttons
BTN_A = 0   # Land
BTN_B = 1   # Reset
BTN_Y = 4   # Takeoff
BTN_RB = 7  # Mode switch
BTN_LB = 6  # Coordinate frame switch


class ControlMode(IntEnum):
    ATTITUDE = 0
    VELOCITY = 1
    POSITION = 2


def apply_deadzone(val: float, dz: float = DEADZONE) -> float:
    if abs(val) < dz:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - dz) / (1.0 - dz)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


class XboxTeleop:
    def __init__(self) -> None:
        self.mode = ControlMode.ATTITUDE

        self.max_angle_deg = rospy.get_param("~max_angle_deg", 30.0)
        self.max_yaw_rate_dps = rospy.get_param("~max_yaw_rate_dps", 120.0)
        self.max_thrust_n = rospy.get_param("~max_thrust_n", 16.7)
        self.vel_scale = rospy.get_param("~vel_scale", 1.0)
        self.pos_rate = rospy.get_param("~pos_rate", 0.05)
        self.yaw_deg_rate = rospy.get_param("~yaw_deg_rate", 2.0)
        self.takeoff_z = rospy.get_param("~takeoff_z", 1.0)
        self.joy_timeout = rospy.get_param("~joy_timeout", 1.0)
        self.frame_world_id = rospy.get_param("~frame_world_id", "world")
        self.autostart_joy = rospy.get_param("~autostart_joy", True)
        self.joy_autorepeat_rate = rospy.get_param("~joy_autorepeat_rate", 20.0)

        self.attitude_topic = rospy.get_param("~attitude_topic", "/sim/setpoint_raw/attitude")
        self.velocity_topic = rospy.get_param("~velocity_topic", "/sim/setpoint_velocity/cmd_vel")
        self.position_topic = rospy.get_param("~position_topic", "/sim/setpoint_position/local")
        self.reset_topic = rospy.get_param("~reset_topic", "/sim/reset")
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")

        self.att_pub = rospy.Publisher(self.attitude_topic, Float32MultiArray, queue_size=1)
        self.vel_pub = rospy.Publisher(self.velocity_topic, TwistStamped, queue_size=1)
        self.pos_pub = rospy.Publisher(self.position_topic, PoseStamped, queue_size=1)
        self.reset_pub = rospy.Publisher(self.reset_topic, Empty, queue_size=1, latch=True)

        self.joy_msg = None
        self.last_joy_time = rospy.Time(0)
        self._warned_no_joy = False
        self._prev_buttons = {}
        self.lb_world = False
        self._joy_proc = None

        # Setpoints
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.thrust = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate_dps = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw_deg = 0.0
        self._last_update_time = None

        rospy.Subscriber(self.joy_topic, Joy, self._joy_cb, queue_size=1)
        self._maybe_start_joy_node()
        rospy.on_shutdown(self._stop_joy_node)

    def _is_joy_available(self) -> bool:
        try:
            master = rosgraph.Master(rospy.get_name())
            topics = master.getPublishedTopics("")
        except Exception:
            return False
        for name, _ in topics:
            if name == self.joy_topic:
                return True
        return False

    def _maybe_start_joy_node(self) -> None:
        if not self.autostart_joy:
            return
        if self._is_joy_available():
            return
        cmd = [
            "rosrun",
            "joy",
            "joy_node",
            f"_autorepeat_rate:={self.joy_autorepeat_rate}",
        ]
        try:
            preexec = os.setsid if hasattr(os, "setsid") else None
            self._joy_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=preexec,
            )
            rospy.loginfo("Started joy_node (autorepeat_rate=%.1f).", self.joy_autorepeat_rate)
        except Exception as exc:
            self._joy_proc = None
            rospy.logwarn("Failed to start joy_node: %s", exc)

    def _stop_joy_node(self) -> None:
        proc = self._joy_proc
        if proc is None:
            return
        self._joy_proc = None
        if proc.poll() is not None:
            return
        try:
            if hasattr(os, "killpg"):
                os.killpg(proc.pid, signal.SIGTERM)
            else:
                proc.terminate()
        except Exception:
            pass

    def _joy_cb(self, msg: Joy) -> None:
        self.joy_msg = msg
        self.last_joy_time = rospy.Time.now()
        self._warned_no_joy = False

    def _axis(self, idx: int) -> float:
        if self.joy_msg is None or idx >= len(self.joy_msg.axes):
            return 0.0
        return self.joy_msg.axes[idx]

    def _button(self, idx: int) -> int:
        if self.joy_msg is None or idx >= len(self.joy_msg.buttons):
            return 0
        return self.joy_msg.buttons[idx]

    def _reset_setpoints(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.thrust = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.yaw_rate_dps = 0.0
        self.x = self.y = 0.0
        self.yaw_deg = 0.0
        self.z = self.takeoff_z if self.mode == ControlMode.POSITION else 0.0

    def _switch_mode(self) -> None:
        modes = list(ControlMode)
        idx = (modes.index(self.mode) + 1) % len(modes)
        self.mode = modes[idx]
        self._reset_setpoints()
        rospy.loginfo("Mode: %s", self.mode.name)

    def _takeoff(self) -> None:
        self.mode = ControlMode.POSITION
        self.x = 0.0
        self.y = 0.0
        self.z = self.takeoff_z
        self.yaw_deg = 0.0
        rospy.loginfo("Takeoff")

    def _land(self) -> None:
        self.mode = ControlMode.POSITION
        self.z = 0.0
        rospy.loginfo("Land")

    def _reset(self) -> None:
        self.mode = ControlMode.ATTITUDE
        self._reset_setpoints()
        self.reset_pub.publish(Empty())
        rospy.loginfo("Reset")

    def _handle_buttons(self) -> None:
        self.lb_world = bool(self._button(BTN_LB))
        for btn in (BTN_A, BTN_B, BTN_Y, BTN_RB):
            pressed = bool(self._button(btn))
            was = self._prev_buttons.get(btn, False)
            if pressed and not was:
                if btn == BTN_RB:
                    self._switch_mode()
                elif btn == BTN_Y:
                    self._takeoff()
                elif btn == BTN_A:
                    self._land()
                elif btn == BTN_B:
                    self._reset()
            self._prev_buttons[btn] = pressed

    def _update_attitude(self, left_x: float, left_y: float, right_x: float, right_y: float, dt: float) -> None:
        self.roll = max(-1.0, min(1.0, -right_x)) * self.max_angle_deg
        self.pitch = max(-1.0, min(1.0, right_y)) * self.max_angle_deg

        yaw_rate_dps = max(-1.0, min(1.0, left_x)) * self.max_yaw_rate_dps
        self.yaw_deg = (self.yaw_deg + yaw_rate_dps * dt + 180.0) % 360.0 - 180.0

        thrust_norm = max(0.0, min(1.0, (left_y + 1.0) * 0.5))
        self.thrust = thrust_norm * self.max_thrust_n

    def _update_velocity(self, left_x: float, left_y: float, right_x: float, right_y: float) -> None:
        self.vx = right_y * self.vel_scale
        self.vy = right_x * self.vel_scale
        self.vz = left_y * self.vel_scale
        self.yaw_rate_dps = left_x * self.max_yaw_rate_dps

    def _update_position(self, left_x: float, left_y: float, right_x: float, right_y: float) -> None:
        yaw_rad = math.radians(self.yaw_deg)
        body_dx = right_y * self.pos_rate
        body_dy = right_x * self.pos_rate

        self.x += body_dx * math.cos(yaw_rad) - body_dy * math.sin(yaw_rad)
        self.y += body_dx * math.sin(yaw_rad) + body_dy * math.cos(yaw_rad)
        self.z += left_y * self.pos_rate
        self.z = max(0.0, self.z)

        self.yaw_deg += left_x * self.yaw_deg_rate
        self.yaw_deg = ((self.yaw_deg + 180.0) % 360.0) - 180.0

    def update(self) -> None:
        if self.joy_msg is None:
            return

        left_x = apply_deadzone(self._axis(AXIS_LEFT_X))
        left_y = apply_deadzone(self._axis(AXIS_LEFT_Y))
        right_x = apply_deadzone(self._axis(AXIS_RIGHT_X))
        right_y = apply_deadzone(self._axis(AXIS_RIGHT_Y))

        self._handle_buttons()

        now = rospy.Time.now()
        if self._last_update_time is None:
            dt = 1.0 / CONTROL_RATE_HZ
        else:
            dt = max(0.0, (now - self._last_update_time).to_sec())
        self._last_update_time = now

        if self.mode == ControlMode.ATTITUDE:
            self._update_attitude(left_x, left_y, right_x, right_y, dt)
        elif self.mode == ControlMode.VELOCITY:
            self._update_velocity(left_x, left_y, right_x, right_y)
        else:
            self._update_position(left_x, left_y, right_x, right_y)

    def _publish_attitude(self) -> None:
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw_deg)
        w, x, y, z = quat_from_euler(roll_rad, pitch_rad, yaw_rad)

        msg = Float32MultiArray()
        type_mask = 1 | 2 | 4  # Ignore roll/pitch/yaw rates
        msg.data = [
            float(type_mask),
            float(w),
            float(x),
            float(y),
            float(z),
            0.0,
            0.0,
            0.0,
            float(self.thrust),
        ]
        self.att_pub.publish(msg)

    def _publish_velocity(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_world_id if self.lb_world else "base_link"
        msg.twist.linear.x = float(self.vx)
        msg.twist.linear.y = float(self.vy)
        msg.twist.linear.z = float(self.vz)
        msg.twist.angular.z = math.radians(self.yaw_rate_dps)
        self.vel_pub.publish(msg)

    def _publish_position(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = float(self.z)

        yaw_rad = math.radians(self.yaw_deg)
        w, x, y, z = quat_from_euler(0.0, 0.0, yaw_rad)
        msg.pose.orientation.w = float(w)
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)

        self.pos_pub.publish(msg)

    def publish(self) -> None:
        if self.mode == ControlMode.ATTITUDE:
            self._publish_attitude()
        elif self.mode == ControlMode.VELOCITY:
            self._publish_velocity()
        else:
            self._publish_position()

    def spin(self) -> None:
        rate_hz = rospy.get_param("~rate_hz", CONTROL_RATE_HZ)
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.joy_msg is None or (rospy.Time.now() - self.last_joy_time).to_sec() > self.joy_timeout:
                if not self._warned_no_joy:
                    rospy.logwarn("No /joy input received; not publishing setpoints.")
                    self._warned_no_joy = True
                rate.sleep()
                continue

            self.update()
            self.publish()
            rate.sleep()


def main() -> None:
    rospy.init_node("elec5660_xbox_teleop")
    teleop = XboxTeleop()
    teleop.spin()


if __name__ == "__main__":
    main()
