#!/bin/bash
source devel/setup.bash

roslaunch sim_adaptor sim_adaptor.launch use_joy:=true & sleep 2;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch traj_generator traj_test.launch & sleep 2;

wait;
