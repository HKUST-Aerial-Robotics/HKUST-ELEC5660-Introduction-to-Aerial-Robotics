#pragma once

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern int ROW, COL;
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::vector<std::string> CAM_NAMES;
extern double MAX_FREQ;
extern int MAX_CNT;
extern int MIN_CNT;
extern int MIN_DIST;
extern double TRANSLATION_THRESHOLD;
extern double ROTATION_THRESHOLD;
extern double FEATURE_THRESHOLD;
extern int SHOW_FEATURE;
extern int FLOW_BACK;

void readParameters(std::string config_file);
