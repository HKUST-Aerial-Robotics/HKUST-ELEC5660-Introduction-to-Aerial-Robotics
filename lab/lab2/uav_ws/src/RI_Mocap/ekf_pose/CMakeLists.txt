cmake_minimum_required(VERSION 2.8.3)
project(ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

catkin_package(
    )

# add_executable(ekf src/ekf_node.cpp src/conversion.cpp)  # to time synchronization  #oldest one, Euler transform is pnp the world in cam transforms to imu in world
# add_executable(ekf src/ekf_node_mocap.cpp src/conversion.cpp)  # Euler transform is rigidbody in world transforms to imu in world
# add_executable(ekf src/ekf_node_vio.cpp src/conversion.cpp) # only use ekf_node_vio.cpp (ekf_node_mocap.cpp is just for mocap format, base on this one)
# add_executable(ekf src/ekf_node_vio_timesync.cpp src/conversion.cpp)
add_executable(ekf src/ekf_node_vio_timesync_with_acc_pub.cpp src/conversion.cpp)
add_executable(imu_add_noise src/imu_add_noise.cpp src/conversion.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES} )
target_link_libraries(imu_add_noise ${catkin_LIBRARIES} )

