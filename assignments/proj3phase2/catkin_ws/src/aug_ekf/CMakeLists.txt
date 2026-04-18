cmake_minimum_required(VERSION 2.8.3)
project(aug_ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    stereo_vo
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS stereo_vo
    )

add_executable(${PROJECT_NAME}
    src/ekf_node.cpp
    src/ekf_filter.cpp
    src/ekf_model.cpp
    )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} -ldw)

