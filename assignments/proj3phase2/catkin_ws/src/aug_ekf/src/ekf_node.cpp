#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include <ekf_filter.h>

// for debug
// #include <backward.hpp>
// namespace backward {
// backward::SignalHandling sh;
// }

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  ros::init(argc, argv, "aug_ekf");
  ros::NodeHandle n("~");

  ekf_imu_vision::EKFImuVision aug_ekf;
  aug_ekf.init(n);

  ros::spin();
}
