#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

ros::Publisher position_cmd_pub;

ros::Publisher current_pose_pub;

ros::Subscriber uav_pos_sub;

ros::Subscriber trigger_sub;

int traj_id_send = 0;
bool is_init = 0;
bool is_traj = 0;

Vector3d hover_position;
Vector3d current_position;

double hover_yaw = 0.0;
double now_yaw = 0.0;
double now_pitch = 0.0;

double total_pitch = 0.0;
int num_pitch = 0;
bool display_pitch = false;

double odom_time = 0.0;
double hover_start_time = 0.0;

double acc_time = 0.0;
double acc_dist = 0.0;
double linear_time = 0.0;
double linear_dist = 0.0;

double LINEAR_ACC = 0.0;

double LINEAR_VEL = 0.0;

double TRAJ_LENGTH = 0.0;

#define ACC_INTERVAL 0.1;
#define VEL_INTERVAL 0.1; 

FILE * log_file;

void uav_pos_call_back(const nav_msgs::Odometry &msg)
{
	current_position(0) = msg.pose.pose.position.x;
	current_position(1) = msg.pose.pose.position.y;
	current_position(2) = msg.pose.pose.position.z;

	odom_time = msg.header.stamp.toSec();

	now_yaw = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));
	now_pitch = asin(-2.0*(msg.pose.pose.orientation.x*msg.pose.pose.orientation.z - msg.pose.pose.orientation.w*msg.pose.pose.orientation.y));

	if (is_init)
	{
		if (is_traj)
		{

			double des_x = hover_position(0);
			double des_y = hover_position(1);
			double des_z = hover_position(2);

			double des_vx = 0.0;
			double des_vy = 0.0;
			double des_vz = 0.0;

			double des_ax = 0.0;
			double des_ay = 0.0;
			double des_az = 0.0;

			double des_yaw = 0.0;
			
			double dt = odom_time - hover_start_time;

			if (dt >= 0.0 && dt < acc_time)
			{
				des_x = 0.5 * LINEAR_ACC * dt * dt + hover_position(0);
				des_vx = LINEAR_ACC * dt;
				des_ax = LINEAR_ACC;
			}
			else if (dt >= acc_time && dt < acc_time + linear_time)
			{
				des_x = 0.5 * LINEAR_ACC * acc_time * acc_time + LINEAR_VEL * (dt - acc_time) + hover_position(0);
				des_vx = LINEAR_VEL;
				des_ax = 0.0;

				if( dt >= acc_time + 0.5){
					total_pitch += now_pitch;
					num_pitch++;
					display_pitch = true;
				}

			}
			else if (dt >= acc_time + linear_time && dt < 2.0 * acc_time + linear_time)
			{
				des_x = 0.5 * LINEAR_ACC * acc_time * acc_time + LINEAR_VEL * linear_time + LINEAR_VEL * (dt - acc_time - linear_time) - 0.5 * LINEAR_ACC * pow(dt - acc_time - linear_time, 2) + hover_position(0);
				des_vx = LINEAR_VEL - LINEAR_ACC * (dt - acc_time - linear_time);
				des_ax = -LINEAR_ACC;
			}

			quadrotor_msgs::PositionCommand position_cmd;
			position_cmd.header.stamp = msg.header.stamp;
			position_cmd.header.frame_id = "world";

			if (odom_time - hover_start_time <= 2.0 * acc_time + linear_time)
			{
				position_cmd.position.x = des_x;
				position_cmd.position.y = des_y;
				position_cmd.position.z = des_z;
				position_cmd.velocity.x = des_vx;
				position_cmd.velocity.y = des_vy;
				position_cmd.velocity.z = des_vz;
				position_cmd.acceleration.x = des_ax;
				position_cmd.acceleration.y = des_ay;
				position_cmd.acceleration.z = des_az;
				position_cmd.yaw = des_yaw;
				position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
				position_cmd.trajectory_id = traj_id_send;
			}
			else
			{
				position_cmd.position.x = current_position(0);
				position_cmd.position.y = current_position(1);
				position_cmd.position.z = current_position(2);
				position_cmd.velocity.x = 0;
				position_cmd.velocity.y = 0;
				position_cmd.velocity.z = 0;
				position_cmd.acceleration.x = 0;
				position_cmd.acceleration.y = 0;
				position_cmd.acceleration.z = 0;
				position_cmd.yaw = now_yaw;
				position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
				position_cmd.trajectory_id = traj_id_send;

				if(display_pitch){
					double mean_pitch = total_pitch/num_pitch;
					ROS_WARN("pitch info:");
					cout<<"vel: "<<LINEAR_VEL<<endl;
					cout<<"mean pitch: "<<mean_pitch<<endl;
					cout<<"mean pitch in degree: "<<mean_pitch/M_PI * 180<<endl;
					cout<<"num pitch data: "<<num_pitch<<endl;
					display_pitch = false;

					if (log_file != nullptr) {
						fprintf(
							//0  1 2  3  4  5  6  7   8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30
						log_file, "%f,%f,%f,%f,%d\n",
						LINEAR_VEL,linear_time,mean_pitch,mean_pitch/M_PI * 180,num_pitch
						);
						fflush(log_file);
					}
					
				}
			}

			position_cmd_pub.publish(position_cmd);

			position_cmd.position.x = msg.pose.pose.position.x;
			position_cmd.position.y = msg.pose.pose.position.y;
			position_cmd.position.z = msg.pose.pose.position.z;
			position_cmd.velocity.x = msg.twist.twist.linear.x;
			position_cmd.velocity.y = msg.twist.twist.linear.y;
			position_cmd.velocity.z = msg.twist.twist.linear.z;
			position_cmd.acceleration.x = msg.twist.twist.angular.x;
			position_cmd.acceleration.y = msg.twist.twist.angular.y;
			position_cmd.acceleration.z = msg.twist.twist.angular.z;
			position_cmd.yaw = now_yaw;

			current_pose_pub.publish(position_cmd);
		}
	}
	else
	{
		is_init = true;
	}
}

void trigger_callback(const geometry_msgs::PoseStamped::ConstPtr &trigger_msg)
{
	if (is_init)
	{
		std::cout << "[#INFO] get traj trigger info." << std::endl;
		traj_id_send = trigger_msg->header.seq + 1;
		is_traj = true;

		LINEAR_VEL += VEL_INTERVAL;
		LINEAR_ACC += ACC_INTERVAL;

		acc_time = LINEAR_VEL / LINEAR_ACC;
		acc_dist = 0.5 * LINEAR_VEL * LINEAR_VEL / LINEAR_ACC;

		linear_dist = TRAJ_LENGTH - 2.0 * acc_dist;
		linear_time = linear_dist / LINEAR_VEL;

		cout << "acc_time: " << acc_time << endl;
		cout << "acc_dist: " << acc_dist << endl;
		cout << "linear_time: " << linear_time << endl;
		cout << "linear_dist: " << linear_dist << endl;

		if (linear_dist < 1.0)
		{
			ROS_ERROR("linear distance not enough!!!!");
			return;
		}

		hover_position = current_position;
		hover_yaw = now_yaw;

		total_pitch = 0.0;
		num_pitch = 0;

		hover_start_time = odom_time;

		cout << "line start:\n"
			 << hover_position.transpose() << "\n end: " << (hover_position + Vector3d(TRAJ_LENGTH, 0, 0)).transpose() << "\n\n\n\n";
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "line_test");
	ros::NodeHandle nh;

	nh.param("line_test/LINEAR_ACC", LINEAR_ACC, 1.0);
	nh.param("line_test/LINEAR_VEL", LINEAR_VEL, 1.0);
	nh.param("line_test/TRAJ_LENGTH", TRAJ_LENGTH, 7.0);

	uav_pos_sub = nh.subscribe("/vins_estimator/imu_propagate", 10, uav_pos_call_back);
	trigger_sub = nh.subscribe("/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
	current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose", 1);

	char file_name[80] = {0};
	sprintf(file_name, "/home/lwangax/ctrl_ws/log/start_v_%d.csv", static_cast<int>(round(LINEAR_VEL*10)));
	log_file = fopen(file_name,"w");
	if (log_file != nullptr) {
		ROS_INFO("Log inited");
	} else {
		ROS_ERROR("Can't open log file");
	}

	LINEAR_VEL -= VEL_INTERVAL;
	LINEAR_ACC -= ACC_INTERVAL;

	ros::spin();
	return 0;
}
