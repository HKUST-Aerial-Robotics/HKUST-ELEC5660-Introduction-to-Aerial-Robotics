#include <ekf_filter.h>

namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}

void EKFImuVision::init(ros::NodeHandle& nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt1_.setZero();
  Rt2_.setZero();

  // addition and removel of augmented state

  // TODO
  // set M_a_ and M_r_
  // M_a_ should add the copied keyframe pose to the 15-state EKF state.
  // M_r_ should remove the copied keyframe pose when a new keyframe is selected.
  M_a_.setZero();
  M_r_.setZero();

  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
    node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }

  init_ = false;
  current_imu_ut.setZero();
  path_.poses.clear();

  for (int i = 0; i < 4; i++) latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ =
      node_.subscribe<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/imu", 100, &EKFImuVision::imuCallback, this);
  pnp_sub_ = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
  // opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
  // &EKFImuVision::opticalCallback, this);
  stereo_sub_ = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
  path_pub_ = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {
  // TODO
  // construct a new state using the absolute measurement from marker PnP and insert the new state into the queue

  AugState new_state;
  new_state.time_stamp = msg->header.stamp;
  new_state.key_frame_time_stamp = ros::Time(0);
  new_state.type = pnp;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {
  // TODO
  // label the previous keyframe
  // construct a new state using the relative measurement from VO and insert the new state into the queue

  AugState new_state;
  new_state.time_stamp = msg->header.stamp;
  new_state.key_frame_time_stamp = msg->key_stamp;
  new_state.type = vo;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  // TODO
  // construct a new state using the IMU input and insert the new state into the queue

  AugState new_state;
  new_state.time_stamp = imu_msg->header.stamp;
  new_state.key_frame_time_stamp = ros::Time(0);
  new_state.type = imu;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {
  // TODO
  // predict by IMU inputs

  (void)ut;
  cur_state.mean = prev_state.mean;
  cur_state.covariance = prev_state.covariance;
  cur_state.key_frame_time_stamp = prev_state.key_frame_time_stamp;
}

void EKFImuVision::updatePnP(AugState& cur_state, AugState& prev_state) {
  // TODO
  // update by marker PnP measurements

  cur_state.mean = prev_state.mean;
  cur_state.covariance = prev_state.covariance;
  cur_state.key_frame_time_stamp = prev_state.key_frame_time_stamp;
}

void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {
  // TODO
  // update by relative pose measurements

  cur_state.mean = prev_state.mean;
  cur_state.covariance = prev_state.covariance;
  cur_state.key_frame_time_stamp = prev_state.key_frame_time_stamp;
}

void EKFImuVision::changeAugmentedState(AugState& state) {
  ROS_ERROR("----------------change keyframe------------------------");

  // TODO
  // change augmented state
  (void)state;
}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {
  // TODO
  // process the new state
  // step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
  // step 2: try to initialize the filter if it is not initialized.
  // step 3: repropagate from the iterator you extracted.
  // step 4: remove the old states.

  (void)new_state;
  (void)change_keyframe;
  ROS_WARN_THROTTLE(2.0, "Augmented EKF scaffold: processNewState is not implemented yet.");
  return false;
}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state) {
  // TODO
  // insert the new state to the queue
  // update the latest_idx of the type of the new state
  // return the iterator point to the new state in the queue

  (void)new_state;
  return aug_state_hist_.end();
}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it, bool& init) {
  // TODO
  // repropagate along the queue from the new input according to the type of the inputs / measurements
  // publish the latest fused odom
  // remember to consider the initialization case

  (void)new_input_it;
  (void)init;
}

void EKFImuVision::removeOldState() {
  // TODO
  // remove the unnecessary old states to prevent the queue from becoming too long
}

void EKFImuVision::publishFusedOdom() {
  if (aug_state_hist_.empty()) return;

  AugState last_state = aug_state_hist_.back();

  double phi, theta, psi;
  phi = last_state.mean(3);
  theta = last_state.mean(4);
  psi = last_state.mean(5);

  if (last_state.mean.head(3).norm() > 20) {
    ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
    return;
  }

  // using the zxy euler angle
  Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = last_state.time_stamp;

  odom.pose.pose.position.x = last_state.mean(0);
  odom.pose.pose.position.y = last_state.mean(1);
  odom.pose.pose.position.z = last_state.mean(2);

  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = last_state.mean(6);
  odom.twist.twist.linear.y = last_state.mean(7);
  odom.twist.twist.linear.z = last_state.mean(8);

  fuse_odom_pub_.publish(odom);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x = last_state.mean(0);
  path_pose.pose.position.y = last_state.mean(1);
  path_pose.pose.position.z = last_state.mean(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);
}

bool EKFImuVision::initFilter() {
  // TODO
  // Initial the filter when a keyframe after marker PnP measurements is available

  ROS_WARN_THROTTLE(2.0, "Augmented EKF scaffold: initFilter is not implemented yet.");
  return false;
}

bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {
  // TODO
  // Initialize the absolute pose of the state in the queue using marker PnP measurement.
  // This is only step 1 of the initialization.

  (void)start_it;
  ROS_WARN_THROTTLE(2.0, "Augmented EKF scaffold: initUsingPnP is not implemented yet.");
  return false;
}

Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}

}  // namespace ekf_imu_vision
