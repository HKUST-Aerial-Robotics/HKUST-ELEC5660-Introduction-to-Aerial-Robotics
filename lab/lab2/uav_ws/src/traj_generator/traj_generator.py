import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from quadrotor_msgs.msg import PositionCommand

class CircleTrajectoryGenerator():
    def __init__(self):
        rospy.init_node("trajectory_generator", anonymous=True)
        # ROS publishers and subscribers
        self._uav_pos_sub = rospy.Subscriber("/ekf/ekf_odom", Odometry, self._uav_pos_callback)
        self._trigger_sub = rospy.Subscriber("/traj_start_trigger", PoseStamped, self._trigger_callback)
        self._position_cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
        self._current_pose_pub = rospy.Publisher('/current_pose', PositionCommand, queue_size=1)
        
        # Parameters
        self._is_init = False
        self._is_traj = False
        self._odom_time = 0.0
        self._start_time = 0.0
        self._hover_pos = []
        self._hover_yaw = 0.0
        # State
        self._cur_pos = []
        self._cur_yaw = 0.0
        # CMD msg info
        self._traj_id = 0
        
        # for circle trajectory
        self._radius = 1.0
        self._period = 10.0
        self._rev    = 1.0
        self._follow_yaw = False

        print("test")

        rospy.spin()

    def _uav_pos_callback(self, msg):
        #print("pos_callback")
        self._odom_time = msg.header.stamp.to_sec()
        self._cur_pos = [msg.pose.pose.position.x, 
                         msg.pose.pose.position.y, 
                         msg.pose.pose.position.z]
        self._cur_yaw = np.math.atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                                           msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                                      1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + 
                                               msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))

        if (self._is_init):
            if (self._is_traj):
                dt = self._odom_time - self._start_time
                is_traj_finished = dt > self._rev * self._period
                dt = min(dt, self._rev * self._period)

                des_x = self._hover_pos[0] + self._radius * np.math.sin(2 * np.math.pi / self._period * (dt))
                des_y = self._hover_pos[1] - self._radius * np.math.cos(2 * np.math.pi / self._period * (dt)) + self._radius
                des_z = self._hover_pos[2]

                des_vx = self._radius * 2 * np.math.pi / self._period * np.math.cos(2 * np.math.pi / self._period * (dt))
                des_vy = self._radius * 2 * np.math.pi / self._period * np.math.sin(2 * np.math.pi / self._period * (dt))
                des_vz = 0.0

                des_ax = -4 * np.math.pi * np.math.pi / self._period / self._period * self._radius * np.math.sin(2 * np.math.pi / self._period * (dt))
                des_ay = 4 * np.math.pi * np.math.pi / self._period / self._period * self._radius * np.math.cos(2 * np.math.pi / self._period * (dt))
                des_az = 0.0

                des_yaw = 0.0
                if (self._follow_yaw):
                    des_yaw = np.math.atan2(des_vy, des_vx)
                    while (des_yaw > np.math.pi):
                        des_yaw -= 2*np.math.pi
                    while (des_yaw < -np.math.pi):
                        des_yaw += 2*np.math.pi

                position_cmd = PositionCommand()
                position_cmd.header.stamp = msg.header.stamp
                position_cmd.header.frame_id = "world"
                if (not is_traj_finished):
                    position_cmd.position.x = des_x
                    position_cmd.position.y = des_y
                    position_cmd.position.z = des_z
                    position_cmd.velocity.x = des_vx
                    position_cmd.velocity.y = des_vy
                    position_cmd.velocity.z = des_vz
                    position_cmd.acceleration.x = des_ax
                    position_cmd.acceleration.y = des_ay
                    position_cmd.acceleration.z = des_az
                    position_cmd.yaw = des_yaw
                    position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY
                    position_cmd.trajectory_id = self._traj_id
                    # publish cmd msg
                    self._position_cmd_pub.publish(position_cmd)
                else:
                    position_cmd.position.x = des_x
                    position_cmd.position.y = des_y
                    position_cmd.position.z = des_z
                    position_cmd.velocity.x = 0.0
                    position_cmd.velocity.y = 0.0
                    position_cmd.velocity.z = 0.0
                    position_cmd.acceleration.x = 0.0
                    position_cmd.acceleration.y = 0.0
                    position_cmd.acceleration.z = 0.0
                    position_cmd.yaw = des_yaw
                    position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED
                    position_cmd.trajectory_id = self._traj_id
                # publish current state
                position_cmd.position.x      = msg.pose.pose.position.x
                position_cmd.position.y      = msg.pose.pose.position.y
                position_cmd.position.z      = msg.pose.pose.position.z
                position_cmd.velocity.x      = msg.twist.twist.linear.x
                position_cmd.velocity.y      = msg.twist.twist.linear.y
                position_cmd.velocity.z      = msg.twist.twist.linear.z
                position_cmd.acceleration.x  = msg.twist.twist.angular.x
                position_cmd.acceleration.y  = msg.twist.twist.angular.y
                position_cmd.acceleration.z  = msg.twist.twist.angular.z
                position_cmd.yaw             = self._cur_yaw
                self._current_pose_pub.publish(position_cmd)
            else:
                pass
        else:
            self._is_init = True
            print("rcv odom")

    def _trigger_callback(self, msg):
        # print("trigger_callback")
        if (self._is_init):
            rospy.loginfo("[#INFO] Get trajectory trigger INFO.")
            self._traj_id = msg.header.seq + 1
            self._is_traj = True

            # update hover state
            self._hover_pos = self._cur_pos
            self._hover_yaw = self._cur_yaw
            self._start_time = self._odom_time

            print("circle")
            rospy.loginfo("[#INFO] Circle start!")
            rospy.loginfo("[#INFO] Start position: [%f,%f,%f].", self._hover_pos[0], self._hover_pos[1], self._hover_pos[2])
            rospy.loginfo("[#INFO] Follow YAW: %d.", self._follow_yaw)


if __name__ == '__main__':
    traj_generator = CircleTrajectoryGenerator()
    


