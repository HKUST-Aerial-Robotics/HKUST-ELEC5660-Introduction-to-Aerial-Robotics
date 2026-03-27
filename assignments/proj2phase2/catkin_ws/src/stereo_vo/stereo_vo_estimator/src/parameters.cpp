#include "parameters.h"

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
int ROW, COL;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::vector<std::string> CAM_NAMES;
int MIN_CNT;
int MAX_CNT;
int MIN_DIST;
double TRANSLATION_THRESHOLD;
double ROTATION_THRESHOLD;
double FEATURE_THRESHOLD;
int FLOW_BACK;
int SHOW_FEATURE;
double MAX_FREQ;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_FREQ = fsSettings["max_freq"];
    MAX_CNT = fsSettings["max_cnt"];
    MIN_CNT = fsSettings["min_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    TRANSLATION_THRESHOLD = fsSettings["translation_threshold"];
    ROTATION_THRESHOLD = fsSettings["rotation_threshold"];
    FEATURE_THRESHOLD = fsSettings["feature_threshold"];
    SHOW_FEATURE = fsSettings["show_feature"];
    FLOW_BACK = fsSettings["flow_back"];


    printf("MAX CNT: %d\n", MAX_CNT);
    printf("MIN CNT: %d\n", MIN_CNT);
    printf("MIN DIST: %d\n", MIN_DIST);
    printf("TRANSLATION THRESHOLD: %f\n", TRANSLATION_THRESHOLD);
    printf("ROTATION THRESHOLD: %f\n", ROTATION_THRESHOLD);
    printf("FLOW BACK: %d\n", FLOW_BACK);
    printf("SHOW FEATURE: %d\n", SHOW_FEATURE);


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    

    cv::Mat cv_T;
    Eigen::Matrix4d T;

    fsSettings["body_T_cam0"] >> cv_T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    fsSettings["body_T_cam1"] >> cv_T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
    
    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib; 
    CAM_NAMES.push_back(cam1Path);


    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    fsSettings.release();
}
