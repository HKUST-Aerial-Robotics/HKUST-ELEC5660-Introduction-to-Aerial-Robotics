#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <sys/time.h>
#include <string>
#include <thread>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

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

std::chrono::time_point<std::chrono::system_clock> hover_start_time; 

double min_freq = 1.0;
double max_freq = 20.0;

double amp = 0.2;

double scan_duration = 30.0;

string scan_dir = "x";

Vector3d scan_dir_mul = Vector3d::UnitX();

void pub_cmd(const std::chrono::time_point<std::chrono::system_clock>& cur_time){
		
				
	double dt = (cur_time - hover_start_time).count() * 1e-9;
	// cout<<"dt: "<<dt<<endl;
	bool traj_finish = dt > scan_duration;

	dt = min(dt, scan_duration);

	double cur_freq = min_freq + dt / scan_duration * (max_freq - min_freq);

	double omega = 2 * M_PI * cur_freq;

	Vector3d des_p = hover_position;

	Vector3d des_v = Vector3d::Zero();
	
	Vector3d des_a = scan_dir_mul * amp * sin(omega * dt);

	double des_yaw = hover_yaw;

	Matrix3d rot_z;
	rot_z << cos(des_yaw), -sin(des_yaw), 0,
			sin(des_yaw),   cos(des_yaw), 0,
			0,                 0, 1;

	des_a = rot_z * des_a;

	// if(follow_yaw){
	// 	des_yaw = atan2(des_vy, des_vx);//(dt) / period * 2.0 * M_PI;
	// 	//if(dt > period) des_yaw = -(dt) / period * 2.0 * M_PI;
	// while(des_yaw > M_PI) des_yaw -= (2*M_PI);
	// while(des_yaw < -M_PI) des_yaw += (2*M_PI);
	// }
	// else{
	// 	des_yaw = hover_yaw;
	// }

	// if(move_z){
	// 	des_z = hover_position(2) + z_diff - z_diff * cos(2 * M_PI / period * (dt));
	// 	des_vz = 2 * M_PI / period * z_diff * sin(2 * M_PI / period * (dt));
	// 	des_az = 4 * M_PI * M_PI / period / period * z_diff * cos(2 * M_PI / period * (dt));
	// }
		
	quadrotor_msgs::PositionCommand position_cmd;
	position_cmd.header.stamp    = ros::Time::now();
	position_cmd.header.frame_id = "world";
					
	if(!traj_finish){
		position_cmd.position.x      = des_p.x();
		position_cmd.position.y      = des_p.y();
		position_cmd.position.z      = des_p.z();
		position_cmd.velocity.x      = des_v.x();
		position_cmd.velocity.y      = des_v.y();
		position_cmd.velocity.z      = des_v.z();
		position_cmd.acceleration.x  = des_a.x();
		position_cmd.acceleration.y  = des_a.y();
		position_cmd.acceleration.z  = des_a.z();
		position_cmd.yaw             = des_yaw;
		position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
		position_cmd.trajectory_id = traj_id_send;

		traj_state2pub.data = 3;
	}
	else{
		position_cmd.position.x      = des_p.x();
		position_cmd.position.y      = des_p.y();
		position_cmd.position.z      = des_p.z();
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

// 	odom_time = msg.header.stamp.toSec();

    now_yaw  = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));
	
	if(is_init){
// 		if(is_traj) {
			
// 			double dt = odom_time - hover_start_time;
// 			bool traj_finish = dt > scan_duration;

// 			dt = min(dt, scan_duration);

// 			double cur_freq = min_freq + dt / scan_duration * (max_freq - min_freq);

// 			double omega = 2 * M_PI * cur_freq;

// 			Vector3d des_p = hover_position;

// 			Vector3d des_v = Vector3d::Zero();
			
// 			Vector3d des_a = scan_dir_mul * sin(omega * dt);

// 		    double des_yaw = hover_yaw;

// 			Matrix3d rot_z;
// 			rot_z << cos(des_yaw), -sin(des_yaw), 0,
// 					sin(des_yaw),   cos(des_yaw), 0,
// 					0,                 0, 1;

// 		    des_a = rot_z * des_a;

// 		    // if(follow_yaw){
// 		    // 	des_yaw = atan2(des_vy, des_vx);//(dt) / period * 2.0 * M_PI;
// 		    // 	//if(dt > period) des_yaw = -(dt) / period * 2.0 * M_PI;
// 			// while(des_yaw > M_PI) des_yaw -= (2*M_PI);
// 			// while(des_yaw < -M_PI) des_yaw += (2*M_PI);
// 		    // }
// 		    // else{
// 		    // 	des_yaw = hover_yaw;
// 		    // }

// 		    // if(move_z){
//             // 	des_z = hover_position(2) + z_diff - z_diff * cos(2 * M_PI / period * (dt));
//             // 	des_vz = 2 * M_PI / period * z_diff * sin(2 * M_PI / period * (dt));
//             // 	des_az = 4 * M_PI * M_PI / period / period * z_diff * cos(2 * M_PI / period * (dt));
//             // }
                
//             quadrotor_msgs::PositionCommand position_cmd;
//             position_cmd.header.stamp    = msg.header.stamp;
//             position_cmd.header.frame_id = "world";
                               
//             if(!traj_finish){
// 	        	position_cmd.position.x      = des_p.x();
// 	            position_cmd.position.y      = des_p.y();
// 	            position_cmd.position.z      = des_p.z();
// 	            position_cmd.velocity.x      = des_v.x();
// 	            position_cmd.velocity.y      = des_v.y();
// 	            position_cmd.velocity.z      = des_v.z();
// 	            position_cmd.acceleration.x  = des_a.x();
// 	            position_cmd.acceleration.y  = des_a.y();
// 	            position_cmd.acceleration.z  = des_a.z();
// 	            position_cmd.yaw             = des_yaw;
// 	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
// 	            position_cmd.trajectory_id = traj_id_send;

// 				traj_state2pub.data = 3;
//         	}
//         	else{
//         		position_cmd.position.x      = des_p.x();
// 	            position_cmd.position.y      = des_p.y();
// 	            position_cmd.position.z      = des_p.z();
// 	            position_cmd.velocity.x      = 0;
// 	            position_cmd.velocity.y      = 0;
// 	            position_cmd.velocity.z      = 0;
// 	            position_cmd.acceleration.x  = 0;
// 	            position_cmd.acceleration.y  = 0;
// 	            position_cmd.acceleration.z  = 0;
// 	            position_cmd.yaw             = des_yaw;
// 	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
// 	            position_cmd.trajectory_id = traj_id_send;   
				
// 				is_traj = false;

// 				traj_state2pub.data = 4;  		
//         	}

// 			position_cmd_pub.publish( position_cmd );
                

//             position_cmd.position.x      = msg.pose.pose.position.x;
//             position_cmd.position.y      = msg.pose.pose.position.y;
//             position_cmd.position.z      = msg.pose.pose.position.z;
//             position_cmd.velocity.x      = msg.twist.twist.linear.x;
//             position_cmd.velocity.y      = msg.twist.twist.linear.y;
//             position_cmd.velocity.z      = msg.twist.twist.linear.z;
//             position_cmd.acceleration.x  = msg.twist.twist.angular.x;
//             position_cmd.acceleration.y  = msg.twist.twist.angular.y;
//             position_cmd.acceleration.z  = msg.twist.twist.angular.z;
//             position_cmd.yaw             = now_yaw;

//             current_pose_pub.publish( position_cmd );

//         }

// 		traj_state_pub.publish(traj_state2pub);
    }
    else{
        is_init = true;
    }
}

void trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg ){
    if ( is_init ){
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        traj_id_send= trigger_msg->header.seq + 1;

		traj_state2pub.data = 2;
		traj_state_pub.publish(traj_state2pub);

		int wait_ms = 15;
		
		std::chrono::milliseconds dura(wait_ms);
        std::this_thread::sleep_for(dura);

        hover_position = current_position;
        hover_yaw = now_yaw;

        hover_start_time = std::chrono::system_clock::now();
		is_traj    = true;

		cout<<"-----------------"<<endl;
		ROS_WARN("scan start!");
        cout<<"start position:\n"<< hover_position.transpose()<<"\n amp: "<<amp<<"\n scan_dir: "<< scan_dir<<"\n scan_duration: "<<scan_duration<<endl;
        cout<<"min_freq: "<<min_freq<<"\n max_freq: "<< max_freq<<endl;
    }
}

void cmd_pub_loop(){

    const std::chrono::duration<double> max_update_interval(1.0 / 200.0);

    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::duration<double> process_time;

	ROS_INFO("start cmd pub loop");

    while(true){
		start_time = std::chrono::system_clock::now();
		if(is_init){
			if(is_traj){
				pub_cmd(start_time);
			}
		}

		process_time = std::chrono::system_clock::now() - start_time;
		// cout<<"process_time: "<<process_time.count()<<endl;

		if(process_time < max_update_interval){
			this_thread::sleep_for(max_update_interval - process_time);
		}

    }
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "circle_test");
	ros::NodeHandle nh("~");
	std::thread cmd_pub_thread(cmd_pub_loop);
	
	nh.param("min_freq", min_freq, 1.0);
	nh.param("max_freq", max_freq, 20.0);
	
	nh.param("amp", amp, 1.0);

	nh.param("scan_duration", scan_duration, 30.0);

	nh.param("scan_dir", scan_dir, string("x"));

	if(scan_dir == string("x")){
		scan_dir_mul = Vector3d::UnitX();
	}
	else if(scan_dir == string("y")){
		scan_dir_mul = Vector3d::UnitY();
	}
	else if(scan_dir == string("z")){
		scan_dir_mul = Vector3d::UnitZ();
	}
	else{
		ROS_ERROR("Unsupported direction, use x dir");
		scan_dir_mul = Vector3d::UnitX();
	}

	min_freq = max(min_freq, 0.0);
	max_freq = max(max_freq, min_freq);

	// amp = min(max(amp, 0.0), 1.0);

	uav_pos_sub = nh.subscribe("odom", 100,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
	current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose",1);

	traj_state_pub = nh.advertise<std_msgs::Int16>("traj_state", 10);

	traj_state2pub.data = 0;

	ros::spin();
	return 0;
}
