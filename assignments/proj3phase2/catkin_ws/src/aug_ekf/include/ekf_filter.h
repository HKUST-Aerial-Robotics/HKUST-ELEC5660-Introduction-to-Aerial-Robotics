#ifndef _EKF_IMU_VISION_H_
#define _EKF_IMU_VISION_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ekf_model.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <deque>

#include <stereo_vo/relative_pose.h>

using namespace std;

namespace ekf_imu_vision {

  enum {imu, pnp, vo, keyframe};

struct AugState {
  int type;                       // the type of the state: imu, pnp, vo, keyframe
  ros::Time time_stamp;           // time stamp
  ros::Time key_frame_time_stamp; // the keyframe time stamp of the state

  Vec(21) mean;                   // estimated mean of the state
                                  //  1: x0:2 ~ x, y, z """
                                  //  2: x3:5 ~ phi theta psi """
                                  //  3: x6:8 ~ vx vy vz """
                                  //  4: x9:11 ~ bgx bgy bgz """
                                  //  5: x12:14 ~  bax bay baz """
                                  //  6: x15:17 ~ keyframe x, y, z """
                                  //  7: x18:20 ~ keyframe phi theta psi """

  Mat(21, 21) covariance;         // covariance of the state
  Vec6 ut;                        // input or measurement of the state
};

class EKFImuVision {
private:
  /* ============================== EKF base ============================== */

  deque<AugState> aug_state_hist_;  // the queue storing the necessary state history in chronological order
  unsigned int latest_idx[4];       //the index of the latest state of the 4 type in the queue
  Vec6 current_imu_ut;

  /* ---------- parameter ---------- */
  Mat12x12        Qt_;         // imu
  Mat6x6          Rt1_;        // pnp
  Mat6x6          Rt2_;        // stereo vo

  Mat(21, 15) M_a_;
  Mat(15, 21) M_r_;

  bool initFilter();
  bool initUsingPnP(deque<AugState>::iterator start_it);

  void predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut);
  void updatePnP(AugState& cur_state, AugState& prev_state);
  void updateVO(AugState& cur_state, AugState& prev_state);
  void changeAugmentedState(AugState& state);

  bool processNewState(AugState& new_state, bool change_keyframe);
  deque<AugState>::iterator insertNewState(AugState& new_state);
  void repropagate(deque<AugState>::iterator& new_input_it,  bool& during_init);
  void removeOldState();

  Vec3 rotation2Euler(const Mat3x3& R);

  /* ---------- flag ---------- */
  bool            init_;

  /* ============================== ros interface ============================== */
  ros::NodeHandle node_;
  ros::Subscriber imu_sub_, pnp_sub_, stereo_sub_;
  ros::Publisher  fuse_odom_pub_, path_pub_;
  nav_msgs::Path  path_;

  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void PnPCallback(const nav_msgs::OdometryConstPtr& msg);
  void stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg);
  void publishFusedOdom();

public:
  EKFImuVision(/* args */);
  ~EKFImuVision();

  void init(ros::NodeHandle& nh);
};

// EKFImuVision::
}  // namespace ekf_imu_vision

#endif