/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <deque>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"

using namespace std;
using namespace Eigen;

class Estimator {

public:
  class frame {
    public:
      ros::Time frame_time;     // image time
      cv::Mat img;              // left image
      vector<cv::Point2f> uv;   // 2d feature points of the left image
      vector<cv::Point3f> xyz;  // 3d feature points in the current camera frame
      Matrix3d w_R_c;
      Vector3d w_t_c;           // the current camera pose in the world frame (IMU frame at initialization)
  };

public:

  Estimator();

  void reset();

  void setParameter();

  void readIntrinsicParameter(const vector<string>& calib_file);


  // interface
  bool inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1 = cv::Mat());


  // internal
  bool trackFeatureBetweenFrames(const Estimator::frame& key_frame, const cv::Mat& cur_img,
                                    vector<cv::Point3f>& key_pts_3d, vector<cv::Point2f>& cur_pts_2d);
  bool estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d, vector<cv::Point2f>& cur_pts_2d,
                                 Matrix3d& R, Vector3d& t);

  void extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv);

  bool trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1, vector<cv::Point2f>& left_pts,
                                vector<cv::Point2f>& right_pts);
  void generate3dPoints(const vector<cv::Point2f>& left_pts, const vector<cv::Point2f>& right_pts,
                        vector<cv::Point3f>& cur_pts_3d, vector<cv::Point2f>& cur_pts_2d);


  // utils
  bool inBorder(const cv::Point2f &pt, const int &row, const int &col);

  double distance(cv::Point2f pt1, cv::Point2f pt2);

  template <typename Derived>
  void reduceVector(vector<Derived>& v, vector<uchar> status);

  vector<cv::Point2f> undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam);

  void triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                        Eigen::Vector2d& point0, Eigen::Vector2d& point1, Eigen::Vector3d& point_3d);


  double reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d);

  void updateLatestStates(frame &latest_frame);

  frame key_frame;  // keyframe used for frame to frame pose estimation
  frame prev_frame; // previous frame, may be useful when the current keyframe is not good for estimation

  int fail_cnt;

  bool init_finish;

  Matrix3d ric[2];  // rotation from camera to body (IMU)
  Vector3d tic[2];  // translation from camera to body (IMU)
  Matrix4d Tlr;     // transform from the left camera to the right camera

  vector<camodocal::CameraPtr> m_camera;

  ros::Time latest_time, rel_key_time;  // the time of the current frame and the time of the key frame.

  Eigen::Matrix3d c_R_k;
  Eigen::Vector3d c_t_k;  // transform R|t from the keyframe to the current frame, you should compute it.

  Eigen::Vector3d latest_P;
  Eigen::Quaterniond latest_Q;  // latest pose of the current body (IMU) frame in the world frame (the IMU frame at initialization)

  Eigen::Vector3d latest_rel_P;
  Eigen::Quaterniond latest_rel_Q;  // latest relative pose of the current body frame relative to the body frame of the key frame.

  vector<cv::Point3f> latest_pointcloud;  // latest point cloud in the current camera frame

};