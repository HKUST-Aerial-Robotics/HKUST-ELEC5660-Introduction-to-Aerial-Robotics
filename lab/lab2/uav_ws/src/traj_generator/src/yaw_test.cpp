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

int traj_id_send= 0;
bool is_init= 0;
bool is_traj= 0;

Vector3d hover_position;
Vector3d current_position;

double hover_yaw = 0.0;
double now_yaw = 0.0;

double odom_time = 0.0;
double hover_start_time = 0.0; 

#define yaw_diff 0.5*M_PI

#define step 0

#define period 8.0


void uav_pos_call_back(const nav_msgs::Odometry& msg){
	current_position(0)= msg.pose.pose.position.x;
	current_position(1)= msg.pose.pose.position.y;
	current_position(2)= msg.pose.pose.position.z;

	odom_time = msg.header.stamp.toSec();

    now_yaw  = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));
	
	if(is_init){
		if(is_traj) {
            
            double des_x = hover_position(0);
            double des_y = hover_position(1);
            double des_z = hover_position(2);

            double des_vx = 0;
            double des_vy = 0;
            double des_vz = 0;

		    double des_ax = 0;
		    double des_ay = 0;
		    double des_az = 0;

		    double des_yaw = 0;

		    if(step){
		    	des_yaw = hover_yaw + yaw_diff;
		    }
		    else{
		    	des_yaw = hover_yaw + (odom_time - hover_start_time) / period * 2.0 * M_PI;
		    }
                
            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = msg.header.stamp;
            position_cmd.header.frame_id = "world";
                               
            if(odom_time - hover_start_time < period){
	        	position_cmd.position.x      = des_x;
	            position_cmd.position.y      = des_y;
	            position_cmd.position.z      = des_z;
	            position_cmd.velocity.x      = des_vx;
	            position_cmd.velocity.y      = des_vy;
	            position_cmd.velocity.z      = des_vz;
	            position_cmd.acceleration.x  = des_ax;
	            position_cmd.acceleration.y  = des_ay;
	            position_cmd.acceleration.z  = des_az;
	            position_cmd.yaw             = des_yaw;
	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
	            position_cmd.trajectory_id = traj_id_send;
        	}
        	else{
        		position_cmd.position.x      = des_x;
	            position_cmd.position.y      = des_y;
	            position_cmd.position.z      = des_z;
	            position_cmd.velocity.x      = 0;
	            position_cmd.velocity.y      = 0;
	            position_cmd.velocity.z      = 0;
	            position_cmd.acceleration.x  = 0;
	            position_cmd.acceleration.y  = 0;
	            position_cmd.acceleration.z  = 0;
	            position_cmd.yaw             = now_yaw;
	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
	            position_cmd.trajectory_id = traj_id_send;       		
        	}
                
            position_cmd_pub.publish( position_cmd );

            position_cmd.position.x      = msg.pose.pose.position.x;
            position_cmd.position.y      = msg.pose.pose.position.y;
            position_cmd.position.z      = msg.pose.pose.position.z;
            position_cmd.velocity.x      = msg.twist.twist.linear.x;
            position_cmd.velocity.y      = msg.twist.twist.linear.y;
            position_cmd.velocity.z      = msg.twist.twist.linear.z;
            position_cmd.acceleration.x  = msg.twist.twist.angular.x;
            position_cmd.acceleration.y  = msg.twist.twist.angular.y;
            position_cmd.acceleration.z  = msg.twist.twist.angular.z;
            position_cmd.yaw             = now_yaw;

            current_pose_pub.publish( position_cmd );

        }
    }
    else{
        is_init = true;
    }
}

void trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg ){
    if ( is_init ){
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        traj_id_send= trigger_msg->header.seq + 1;
        is_traj    = true;

        hover_position = current_position;
        hover_yaw = now_yaw;

        hover_start_time = odom_time;

        cout<<"hover at:\n"<< hover_position<<"\n yaw:\n"<<now_yaw + yaw_diff<<"\n\n\n\n";
    }
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "yaw_test");
	ros::NodeHandle nh;
	uav_pos_sub = nh.subscribe("/vins_estimator/imu_propagate", 10,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
	current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose",1);

	ros::spin();
	return 0;
}