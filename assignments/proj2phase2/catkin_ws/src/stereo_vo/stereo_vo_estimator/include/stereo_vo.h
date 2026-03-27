#include <ros/ros.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_vo/relative_pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "estimator.h"


namespace stereo_vo {
    ros::Time bag_start_time;
    bool rcv;

    class stereo_vo {
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    public:
    stereo_vo(ros::NodeHandle nh_) : sync_(MySyncPolicy(100), sub_img0_, sub_img1_) {
        sub_img0_.subscribe(nh_, IMAGE0_TOPIC, 100);
        sub_img1_.subscribe(nh_, IMAGE1_TOPIC, 100);

        std::cout << "IMAGE0_TOPIC:" << IMAGE0_TOPIC << std::endl;
        std::cout << "IMAGE1_TOPIC:" << IMAGE1_TOPIC << std::endl;

        sync_.registerCallback(boost::bind(&stereo_vo::imageCallback, this, _1, _2));

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("Odometry", 100);
        cam_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("Camera_pose", 100);
        path_pub_ = nh_.advertise<nav_msgs::Path>("Path", 100);
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("PointCloud", 100);
        rel_pose_pub_ = nh_.advertise<relative_pose>("Relative_pose", 100);
        rel_pose_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("Relative_pose_vis", 100);

    };

    void pub_odometry();
    void pub_path();
    void pub_pointcloud();

    void imageCallback(const sensor_msgs::ImageConstPtr& img0, const sensor_msgs::ImageConstPtr& img1);

    Estimator estimator;

    private:

    message_filters::Subscriber<sensor_msgs::Image> sub_img0_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img1_;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    ros::Publisher odom_pub_, cam_pose_pub_, path_pub_, point_cloud_pub_, rel_pose_pub_, rel_pose_vis_pub_;
    nav_msgs::Path path_;
    ros::Time image_time_;

    };

}  // namespace stereo_vo