#!/bin/bash

bash -c "roscore" & sleep 5
source /ros2_humble/install/setup.bash
source /ros1_bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics