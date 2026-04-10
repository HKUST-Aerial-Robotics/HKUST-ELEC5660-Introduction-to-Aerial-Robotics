#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Implement EKF prediction here:
}

// Rotation from the camera frame to the IMU frame.
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Implement EKF correction here:
    // Fixed camera/IMU extrinsic used for the conversion:
    // camera origin expressed in the IMU frame = (0.05, 0.05, 0.0)
    // camera-to-IMU rotation = Quaternion(0, 1, 0, 0) in (w, x, y, z) order
    // equivalent rotation matrix:
    //     [ 1,  0,  0]
    //     [ 0, -1,  0]
    //     [ 0,  0, -1]

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q: process noise covariance. Rt: visual measurement noise covariance.
    // You should tune these parameters for a stable filter.
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
