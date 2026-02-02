#!/bin/bash
sudo chmod 777 /dev/ttyTHS0
source devel/setup.bash

roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600  & sleep 5;
roslaunch vrpn_client_ros sample.launch server:=192.168.0.101 & sleep 3;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 3;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 3;
roslaunch ekf mocap.launch & sleep 3;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch traj_generator traj_test.launch & sleep 2;

wait;
