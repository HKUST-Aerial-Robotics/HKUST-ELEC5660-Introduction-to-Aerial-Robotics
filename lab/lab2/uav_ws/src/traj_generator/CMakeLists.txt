cmake_minimum_required(VERSION 2.8.3)
project(traj_generator)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  std_msgs
  tf
)

find_package(Eigen3)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_generator
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/traj_generator.cpp)
add_library(trajectory_generator_waypoint src/trajectory_generator_waypoint.cpp)
target_link_libraries(${PROJECT_NAME} trajectory_generator_waypoint ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(yaw_test src/yaw_test.cpp)
target_link_libraries(yaw_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(circle_test src/circle_test.cpp)
target_link_libraries(circle_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(line_test src/line_test.cpp)
target_link_libraries(line_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(traj_test src/traj_test.cpp)
target_link_libraries(traj_test trajectory_generator_waypoint ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(freq_scan src/freq_scan.cpp)
target_link_libraries(freq_scan ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

add_executable(traj_square src/traj_suqare.cpp)
target_link_libraries(traj_square trajectory_generator_waypoint ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

