#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sys/time.h>
#include <string>
#include <thread>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>

using namespace Eigen;
using namespace std;

ros::Publisher position_cmd_pub;

ros::Publisher current_pose_pub;

ros::Publisher traj_state_pub;

ros::Subscriber uav_pos_sub;

ros::Subscriber trigger_sub;


std_msgs::Int16 traj_state2pub;

int traj_id_send= 0;
bool is_init= 0;
bool is_traj= 0;

Vector3d hover_position;
Vector3d current_position;

double hover_yaw = 0.0;
double now_yaw = 0.0;

double odom_time = 0.0;


double radius = 1.0;
double period = 6.0;
double rev = 2.0;

bool follow_yaw = false;
bool move_z = false;
double z_diff = 0.0;


std::chrono::time_point<std::chrono::system_clock> hover_start_time; 

void pub_cmd(const std::chrono::time_point<std::chrono::system_clock>& cur_time){
	double dt = (cur_time - hover_start_time).count()*1e-9;

	bool traj_finish = dt > rev * period;
	dt = min(dt, rev * period);
	
	double des_x = hover_position(0) + radius * sin(2 * M_PI / period * (dt));
	double des_y = hover_position(1) - radius * cos(2 * M_PI / period * (dt)) + radius;
	double des_z = hover_position(2);


	double des_vx = radius * 2 * M_PI / period * cos(2 * M_PI / period * (dt));
	double des_vy = radius * 2 * M_PI / period * sin(2 * M_PI / period * (dt));
	double des_vz = 0;

	double des_ax = - 4 * M_PI * M_PI / period / period * radius * sin(2 * M_PI / period * (dt));
	double des_ay = 4 * M_PI * M_PI / period / period * radius * cos(2 * M_PI / period * (dt));
	double des_az = 0;

	double des_yaw = 0.0;

	if(follow_yaw){
		des_yaw = atan2(des_vy, des_vx);//(dt) / period * 2.0 * M_PI;
		//if(dt > period) des_yaw = -(dt) / period * 2.0 * M_PI;
	while(des_yaw > M_PI) des_yaw -= (2*M_PI);
	while(des_yaw < -M_PI) des_yaw += (2*M_PI);
	}
	else{
		des_yaw = hover_yaw;
	}

	if(move_z){
		des_z = hover_position(2) + z_diff - z_diff * cos(2 * M_PI / period * (dt));
		des_vz = 2 * M_PI / period * z_diff * sin(2 * M_PI / period * (dt));
		des_az = 4 * M_PI * M_PI / period / period * z_diff * cos(2 * M_PI / period * (dt));
	}
		
	quadrotor_msgs::PositionCommand position_cmd;
	position_cmd.header.stamp    = ros::Time::now();
	position_cmd.header.frame_id = "world";
					
	if(!traj_finish){
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
		/* means finish */
		traj_state2pub.data = 3;
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
		position_cmd.yaw             = des_yaw;
		position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
		position_cmd.trajectory_id = traj_id_send;   
		
		is_traj = false;

		traj_state2pub.data = 4;    		
	}

	position_cmd_pub.publish( position_cmd );
		

	// position_cmd.position.x      = msg.pose.pose.position.x;
	// position_cmd.position.y      = msg.pose.pose.position.y;
	// position_cmd.position.z      = msg.pose.pose.position.z;
	// position_cmd.velocity.x      = msg.twist.twist.linear.x;
	// position_cmd.velocity.y      = msg.twist.twist.linear.y;
	// position_cmd.velocity.z      = msg.twist.twist.linear.z;
	// position_cmd.acceleration.x  = msg.twist.twist.angular.x;
	// position_cmd.acceleration.y  = msg.twist.twist.angular.y;
	// position_cmd.acceleration.z  = msg.twist.twist.angular.z;
	// position_cmd.yaw             = now_yaw;

	// current_pose_pub.publish( position_cmd );
	traj_state_pub.publish(traj_state2pub);

}


void uav_pos_call_back(const nav_msgs::Odometry& msg){
	current_position(0)= msg.pose.pose.position.x;
	current_position(1)= msg.pose.pose.position.y;
	current_position(2)= msg.pose.pose.position.z;

	// odom_time = msg.header.stamp.toSec();

	now_yaw  = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));
	
	if(is_init){
		// if(is_traj) {

		// 	double dt = odom_time - hover_start_time;
		// 	bool traj_finish = dt > rev * period;
		// 	dt = min(dt, rev * period);
			
		// 	double des_x = hover_position(0) + radius * sin(2 * M_PI / period * (dt));
		// 	double des_y = hover_position(1) - radius * cos(2 * M_PI / period * (dt)) + radius;
		// 	double des_z = hover_position(2);


		// 	double des_vx = radius * 2 * M_PI / period * cos(2 * M_PI / period * (dt));
		// 	double des_vy = radius * 2 * M_PI / period * sin(2 * M_PI / period * (dt));
		// 	double des_vz = 0;

		// 	double des_ax = - 4 * M_PI * M_PI / period / period * radius * sin(2 * M_PI / period * (dt));
		// 	double des_ay = 4 * M_PI * M_PI / period / period * radius * cos(2 * M_PI / period * (dt));
		// 	double des_az = 0;

		// 	double des_yaw = 0.0;

		// 	if(follow_yaw){
		// 		des_yaw = atan2(des_vy, des_vx);//(dt) / period * 2.0 * M_PI;
		// 		//if(dt > period) des_yaw = -(dt) / period * 2.0 * M_PI;
		// 	while(des_yaw > M_PI) des_yaw -= (2*M_PI);
		// 	while(des_yaw < -M_PI) des_yaw += (2*M_PI);
		// 	}
		// 	else{
		// 		des_yaw = hover_yaw;
		// 	}

		// 	if(move_z){
		// 		des_z = hover_position(2) + z_diff - z_diff * cos(2 * M_PI / period * (dt));
		// 		des_vz = 2 * M_PI / period * z_diff * sin(2 * M_PI / period * (dt));
		// 		des_az = 4 * M_PI * M_PI / period / period * z_diff * cos(2 * M_PI / period * (dt));
		// 	}
				
		// 	quadrotor_msgs::PositionCommand position_cmd;
		// 	position_cmd.header.stamp    = msg.header.stamp;
		// 	position_cmd.header.frame_id = "world";
							
		// 	if(!traj_finish){
		// 		position_cmd.position.x      = des_x;
		// 		position_cmd.position.y      = des_y;
		// 		position_cmd.position.z      = des_z;
		// 		position_cmd.velocity.x      = des_vx;
		// 		position_cmd.velocity.y      = des_vy;
		// 		position_cmd.velocity.z      = des_vz;
		// 		position_cmd.acceleration.x  = des_ax;
		// 		position_cmd.acceleration.y  = des_ay;
		// 		position_cmd.acceleration.z  = des_az;
		// 		position_cmd.yaw             = des_yaw;
		// 		position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
		// 		position_cmd.trajectory_id = traj_id_send;
		// 	}
		// 	else{
		// 		position_cmd.position.x      = des_x;
		// 		position_cmd.position.y      = des_y;
		// 		position_cmd.position.z      = des_z;
		// 		position_cmd.velocity.x      = 0;
		// 		position_cmd.velocity.y      = 0;
		// 		position_cmd.velocity.z      = 0;
		// 		position_cmd.acceleration.x  = 0;
		// 		position_cmd.acceleration.y  = 0;
		// 		position_cmd.acceleration.z  = 0;
		// 		position_cmd.yaw             = des_yaw;
		// 		position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
		// 		position_cmd.trajectory_id = traj_id_send;   
				
		// 		is_traj = false;    		
		// 	}

		// 	position_cmd_pub.publish( position_cmd );
				

		// 	position_cmd.position.x      = msg.pose.pose.position.x;
		// 	position_cmd.position.y      = msg.pose.pose.position.y;
		// 	position_cmd.position.z      = msg.pose.pose.position.z;
		// 	position_cmd.velocity.x      = msg.twist.twist.linear.x;
		// 	position_cmd.velocity.y      = msg.twist.twist.linear.y;
		// 	position_cmd.velocity.z      = msg.twist.twist.linear.z;
		// 	position_cmd.acceleration.x  = msg.twist.twist.angular.x;
		// 	position_cmd.acceleration.y  = msg.twist.twist.angular.y;
		// 	position_cmd.acceleration.z  = msg.twist.twist.angular.z;
		// 	position_cmd.yaw             = now_yaw;

		// 	current_pose_pub.publish( position_cmd );

		// }
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
		traj_state2pub.data = 2;
		traj_state_pub.publish(traj_state2pub);

		int wait_ms = 15;
		std::chrono::milliseconds dura(wait_ms);
		std::this_thread::sleep_for(dura);

        hover_position = current_position;
        hover_yaw = now_yaw;

        hover_start_time = std::chrono::system_clock::now();
		is_traj = true;

		cout<<"-----------------"<<endl;
		ROS_WARN("circle start!");
        cout<<"start position:\n"<< hover_position.transpose()<<"\n Radius: "<<radius<<"\n rev: "<< rev<<"\n period: "<<period<<endl;
		cout<<"follow yaw: "<<follow_yaw<<endl;

		if(move_z){
			ROS_INFO("move z with diff: %lf", z_diff);
		}
		else{
			ROS_INFO("constant height");
		}
    }
}

void cmd_pub_loop(){
	const std::chrono::duration<double> max_update_interval(1.0/200.0);
	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::duration<double> process_time;
	ROS_INFO ("start cmd pub loop");

	while(true){
		start_time = std::chrono::system_clock::now();
		if(is_init){
			if(is_traj){
				pub_cmd(start_time);
			}
		}
		process_time = std::chrono::system_clock::now() - start_time;

		if(process_time < max_update_interval){
			this_thread::sleep_for(max_update_interval - process_time);
		}
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "circle_test");
	ros::NodeHandle nh("~");
	std::thread cmd_pub_thread(cmd_pub_loop);
	
	nh.param("radius", radius, 1.0);
	nh.param("period", period, 6.0);
	nh.param("rev", rev, 2.0);

	nh.param("follow_yaw", follow_yaw, false);
	nh.param("move_z", move_z, false);
	nh.param("z_diff", z_diff, 0.0);

	uav_pos_sub = nh.subscribe("odom", 100,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
	current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose",1);

	traj_state_pub = nh.advertise<std_msgs::Int16>("traj_state",10);
	traj_state2pub.data = 0;

	ros::spin();
	return 0;
}
