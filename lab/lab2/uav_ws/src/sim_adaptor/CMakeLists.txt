cmake_minimum_required(VERSION 3.0.2)
project(sim_adaptor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_adaptor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
