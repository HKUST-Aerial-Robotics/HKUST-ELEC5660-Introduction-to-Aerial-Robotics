#include "stereo_vo.h"

// using namespace std;

// for debug
// #include <backward.hpp>
// namespace backward {
// backward::SignalHandling sh;
// }

namespace stereo_vo {

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header       = img_msg->header;
    img.height       = img_msg->height;
    img.width        = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step         = img_msg->step;
    img.data         = img_msg->data;
    img.encoding     = "mono8";
    ptr              = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

void stereo_vo::imageCallback(const sensor_msgs::ImageConstPtr& img0,
                              const sensor_msgs::ImageConstPtr& img1) {

  if((img0->header.stamp - image_time_).toSec()< 1.0/MAX_FREQ) return;

  ros::Time tic        = ros::Time::now();
  image_time_ = img0->header.stamp;
  if (estimator.inputImage(image_time_, getImageFromMsg(img0), getImageFromMsg(img1))) {
    pub_odometry();
    pub_path();
    pub_pointcloud();
  }
  std::cout << "image process time: " << (ros::Time::now() - tic).toSec() << std::endl;
}

void stereo_vo::pub_odometry() {

  nav_msgs::Odometry odometry;
  odometry.header.stamp    = estimator.latest_time;
  odometry.header.frame_id = "world";
  odometry.child_frame_id  = "world";

  odometry.pose.pose.position.x    = estimator.latest_P.x();
  odometry.pose.pose.position.y    = estimator.latest_P.y();
  odometry.pose.pose.position.z    = estimator.latest_P.z();
  odometry.pose.pose.orientation.x = estimator.latest_Q.x();
  odometry.pose.pose.orientation.y = estimator.latest_Q.y();
  odometry.pose.pose.orientation.z = estimator.latest_Q.z();
  odometry.pose.pose.orientation.w = estimator.latest_Q.w();

  stereo_vo::stereo_vo::odom_pub_.publish(odometry);

  Eigen::Vector3d    t_wc = estimator.latest_P + estimator.latest_Q.toRotationMatrix() * TIC[0];
  Eigen::Quaterniond q_wc = estimator.latest_Q * Eigen::Quaterniond(RIC[0]);

  geometry_msgs::PoseStamped cam_pose;
  cam_pose.header             = odometry.header;
  cam_pose.pose.position.x    = t_wc.x();
  cam_pose.pose.position.y    = t_wc.y();
  cam_pose.pose.position.z    = t_wc.z();
  cam_pose.pose.orientation.x = q_wc.x();
  cam_pose.pose.orientation.y = q_wc.y();
  cam_pose.pose.orientation.z = q_wc.z();
  cam_pose.pose.orientation.w = q_wc.w();

  stereo_vo::stereo_vo::cam_pose_pub_.publish(cam_pose);

  relative_pose rel_pose;
  rel_pose.header.stamp    = estimator.latest_time;
  rel_pose.header.frame_id = "world";
  rel_pose.key_stamp  = estimator.rel_key_time;

  Eigen::Quaterniond rel_Q = estimator.latest_rel_Q;
      
  Eigen::Vector3d rel_t = estimator.latest_rel_P;

  rel_pose.relative_pose.position.x = rel_t.x();
  rel_pose.relative_pose.position.y = rel_t.y();
  rel_pose.relative_pose.position.z = rel_t.z();

  rel_pose.relative_pose.orientation.w = rel_Q.w();
  rel_pose.relative_pose.orientation.x = rel_Q.x();
  rel_pose.relative_pose.orientation.y = rel_Q.y();
  rel_pose.relative_pose.orientation.z = rel_Q.z();

  rel_pose_pub_.publish(rel_pose);

  geometry_msgs::PoseStamped rel_pose_vis;
  rel_pose_vis.header = rel_pose.header;
  rel_pose_vis.pose   = rel_pose.relative_pose;
  rel_pose_vis_pub_.publish(rel_pose_vis);
}

void stereo_vo::pub_path() {
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x                         = estimator.latest_P.x();
  path_pose.pose.position.y                         = estimator.latest_P.y();
  path_pose.pose.position.z                         = estimator.latest_P.z();
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);
}

void stereo_vo::pub_pointcloud() {
  pcl::PointXYZ                  pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3d pt_tmp;

  for (auto p : estimator.latest_pointcloud) {
    pt_tmp = Vector3d(p.x, p.y, p.z);
    pt_tmp = estimator.latest_Q.toRotationMatrix() * (RIC[0] * pt_tmp + TIC[0]) + estimator.latest_P;
    pt.x   = pt_tmp.x();
    pt.y   = pt_tmp.y();
    pt.z   = pt_tmp.z();
    cloud.push_back(pt);
  }

  cloud.width           = cloud.points.size();
  cloud.height          = 1;
  cloud.is_dense        = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  point_cloud_pub_.publish(cloud_msg);
}

}  // namespace stereo_vo

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_vo_node");
  ros::NodeHandle n("~");

  if (argc != 2) {
    printf("please intput: rosrun stereo_vo stereo_vo [config file] \n"
           "for example: rosrun stereo_vo stereo_vo "
           "src/ELEC5660_proj2_phase3/stereo_vo_estimator/config/realsense/realsense.yaml \n");
    return 1;
  }
  string config_file = argv[1];
  std::cout << "cf:" << config_file << std::endl;

  readParameters(config_file);

  stereo_vo::stereo_vo vo(n);
  vo.estimator.setParameter();

  ros::spin();
  return 0;
}