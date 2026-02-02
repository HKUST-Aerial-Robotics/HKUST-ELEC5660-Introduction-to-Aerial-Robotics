#include "ekf.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include "conversion.h"

using namespace std;
using namespace Eigen;
//X_state: p q v gb ab   with time stamp aligned between imu and img
/*
    EKF model
    prediction:
    xt~ = xt-1 + dt*f(xt-1, ut, 0)
    sigmat~ = Ft*sigmat-1*Ft' + Vt*Qt*Vt'
    Update:
    Kt = sigmat~*Ct'*(Ct*sigmat~*Ct' + Wt*Rt*Wt')^-1
    xt = xt~ + Kt*(zt - g(xt~,0))
    sigmat = sigmat~ - Kt*Ct*sigmat~
*/

// imu frame is imu body frame

//odom: pose px,py pz orientation qw qx qy qz
//imu: acc: x y z gyro: wx wy wz

ros::Publisher odom_pub;
ros::Publisher cam_odom_pub;

//state
geometry_msgs::Pose pose;
Vector3d position, orientation, velocity;

// Now set up the relevant matrices
//states X [p q pdot]  [px,py,pz, wx,wy,wz, vx,vy,vz]
size_t stateSize;                                // x = [p q pdot bg ba]
size_t stateSize_pqv;                                // x = [p q pdot]
size_t measurementSize;                          // z = [p q]
size_t inputSize;                                // u = [w a]
VectorXd X_state(stateSize);                     // x (in most literature)
VectorXd u_input;
VectorXd Z_measurement;                          // z
MatrixXd StateCovariance;                        // sigma
MatrixXd Kt_kalmanGain;                          // Kt
VectorXd X_state_correct(stateSize);                     // x (in most literature)
MatrixXd StateCovariance_correct;                        // sigma
// MatrixXd Ct_stateToMeasurement;                  // Ct
//  VectorXd innovation;                         // z - Hx

MatrixXd Qt;
MatrixXd Rt;
Vector3d u_gyro;
Vector3d u_acc;
Vector3d gravity(0., 0., 9.8); //need to estimate the bias 9.8099
Vector3d bg_0(0., 0., 0); //need to estimate the bias
Vector3d ba_0(0., 0., 0); //need to estimate the bias  0.1
Vector3d ng(0., 0., 0.);
Vector3d na(0., 0., 0.);
Vector3d nbg(0., 0., 0.);
Vector3d nba(0., 0., 0.);

Vector3d q_last;
Vector3d bg_last;
Vector3d ba_last;

//Qt imu covariance matrix  smaller believe system(imu) more
double gyro_cov = 0.01;
double acc_cov = 0.01;
//Rt visual odomtry covariance smaller believe measurement more
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;

double dt = 0.005; //second
double t_last, t_now;  
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool test_odomtag_call = false;
bool odomtag_call = false;

double time_now, time_last;
double time_odom_tag_now;
double diff_time;
//world frame points velocity
vector< pair<VectorXd, sensor_msgs::Imu> > sys_seq;  // keep a sequence of sys imu and X_state(before imu system prediction)
vector<MatrixXd> cov_seq;
void seq_keep(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    #define seqsize 40
    if(sys_seq.size() < seqsize)
    {
        sys_seq.push_back(make_pair(X_state, *imu_msg));
        cov_seq.push_back(StateCovariance);
    }
    else
    {  
        vector< pair<VectorXd, sensor_msgs::Imu> >::iterator k = sys_seq.begin();
        sys_seq.erase(k); //delete the first element
        sys_seq.push_back(make_pair(X_state, *imu_msg));
        vector< MatrixXd >::iterator kk = cov_seq.begin();
        cov_seq.erase(kk); //delete the first element
        cov_seq.push_back(StateCovariance);
    }
}
//choose the coordinate frame imu for the measurement
bool search_proper_frame(double odom_time)
{
    size_t rightframe = sys_seq.size()-1;
    bool find_proper_frame = false;
    for(size_t i = 1; i < sys_seq.size(); i++)
    {
        double time_before = odom_time-sys_seq[i-1].second.header.stamp.toSec();  
        double time_after  = odom_time-sys_seq[i].second.header.stamp.toSec();  
        if((time_before >= 0) && (time_after < 0))
        {
            if(abs(time_before) > abs(time_after))
            {
                rightframe = i;
            }
            else
            {
                rightframe = i-1;
            }
            find_proper_frame = true;
            break;
        }
    }
    if(!find_proper_frame)
    {
        //no process, set the latest one
    }
    //if find, set rightframe as the same time one, if not find, use the lastest one to collect
    //set the right frame as the first frame in the queue
    for(size_t i = 0; i < (rightframe-1); i++)  //keep the rightframe and the one before it
    {
        vector< pair<VectorXd, sensor_msgs::Imu> >::iterator k = sys_seq.begin();
        sys_seq.erase(k); //delete the first element
        vector< MatrixXd >::iterator kk = cov_seq.begin();
        cov_seq.erase(kk); 
    }
    
    if(find_proper_frame)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // seq_keep(msg);
    // nav_msgs::Odometry odom_fusion;
    //your code for propagation
    if(!first_frame_tag_odom)
    {
        if(first_frame_imu)
        {
            first_frame_imu = false;
            time_now = msg->header.stamp.toSec();
            time_last = time_now;
            // imseq_keep(msg);

            system_pub(msg->header.stamp);
        }
        else
        {
            
        }
    }
    
    if(!first_frame_tag_odom)
    { //get the initial pose and orientation in the first frame of measurement 
        if(first_frame_imu)
        {
            first_frame_imu = false;
            time_now = msg->header.stamp.toSec();
            time_last = time_now;
            // imseq_keep(msg);

            system_pub(msg->header.stamp);
        }
        else
        {
            time_now = msg->header.stamp.toSec();
            dt = time_now - time_last;
            // imu_seq_keep(msg);
            if(odomtag_call)
            {
                odomtag_call = false;
                diff_time = time_now - time_odom_tag_now;
                if(diff_time<0)
                {
                    cout << "diff time: " << diff_time << endl;  //???!!! exist !!!???
                    cout << "timeimu: " << time_now - 1.60889e9 << " time_odom: " << time_odom_tag_now - 1.60889e9 << endl;
                    // cout << "diff time: " << diff_time << endl;  //about 30ms
                }
            }
            MatrixXd Ft;
            MatrixXd Vt;

            // u_gyro(0) = msg->angular_velocity.x;
            // u_gyro(1) = msg->angular_velocity.y;
            // u_gyro(2) = msg->angular_velocity.z;
            // u_acc(0)  = msg->linear_acceleration.x;
            // u_acc(1)  = msg->linear_acceleration.y;
            // u_acc(2)  = msg->linear_acceleration.z;
            u_gyro(0) = msg->angular_velocity.x;
            u_gyro(1) = msg->angular_velocity.y;
            u_gyro(2) = msg->angular_velocity.z;
            u_acc(0)  = msg->linear_acceleration.x;
            u_acc(1)  = msg->linear_acceleration.y;
            u_acc(2)  = msg->linear_acceleration.z;

            q_last = X_state.segment<3>(3);  // last X2
            bg_last = X_state.segment<3>(9);  //last X4
            ba_last = X_state.segment<3>(12);  //last X5
            Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
         
            Vt = dt*diff_f_diff_n(q_last);

            X_state += dt*F_model(u_gyro, u_acc);
            StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();
 
            time_last = time_now;
            
            // if(test_odomtag_call) //no frequency boost
            // {
            //     test_odomtag_call = false;
            //     system_pub(msg->header.stamp);
            // }
            system_pub(msg->header.stamp);
        }
    }
  
}

// void re_propagate()
// {
//     MatrixXd Ft;
//     MatrixXd Vt;
//     u_gyro(0) = msg->angular_velocity.x;
//     u_gyro(1) = msg->angular_velocity.y;
//     u_gyro(2) = msg->angular_velocity.z;
//     u_acc(0)  = msg->linear_acceleration.x;
//     u_acc(1)  = msg->linear_acceleration.y;
//     u_acc(2)  = msg->linear_acceleration.z;

//     q_last = X_state.segment<3>(3);  // last X2
//     bg_last = X_state.segment<3>(9);  //last X4
//     ba_last = X_state.segment<3>(12);  //last X5
//     Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
    
//     Vt = dt*diff_f_diff_n(q_last);

//     X_state += dt*F_model(u_gyro, u_acc);
//     StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();
// }

// odom_callback  body frame is the imu frame
// For part 1
// camera position in the IMU frame = (0.05, 0.05, 0)
// camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
//					   RotationMatrix << 1, 0, 0,
//							             0, -1, 0,
//                                       0, 0, -1;

// For part 2 & 3
// camera position in the IMU frame = (-0.1, 0, -0.03)
// camera orientaion:
//                     RotationMatrix << 0, -1, 0,
//                                       -1, 0, 0,
//                                       0, 0, 1;

//Rotation from the camera frame to the IMU frame
Matrix3d Rc_i;     
Vector3d tc_i;  //  cam in imu frame
int cnt = 0;
Vector3d INNOVATION_;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{//assume that the odom_tag from camera is sychronized with the imus and without delay. !!!
    Matrix3d Rw_c;  // world in cam
    Vector3d tw_c;
    // Matrix3d Rc_w;  // cam in world
    // Vector3d tc_w;
    // Matrix3d Rw_i;  // world in imu
    // Vector3d tw_i;
    Matrix3d Ri_w;  //  imu in world
    Vector3d ti_w;
    //your code for update
    if(first_frame_tag_odom)
    {//system begins in first odom frame
        first_frame_tag_odom = false;
        time_odom_tag_now = msg->header.stamp.toSec();

        Vector3d p_temp;
        p_temp(0) = msg->pose.pose.position.x;
        p_temp(1) = msg->pose.pose.position.y;
        p_temp(2) = msg->pose.pose.position.z;
        //quaternion2euler:  ZYX  roll pitch yaw
        Quaterniond q;
        q.w() = msg->pose.pose.orientation.w;
        q.x() = msg->pose.pose.orientation.x;
        q.y() = msg->pose.pose.orientation.y;
        q.z() = msg->pose.pose.orientation.z;
        
        //Euler transform
        Rw_c = q.toRotationMatrix();
        tw_c = p_temp;
        Ri_w = Rw_c.inverse() * Rc_i.inverse();
        ti_w = -( Rw_c.inverse()*tw_c + Ri_w*tc_i );
        Vector3d euler = mat2euler(Ri_w);
        // X_state.segment<3>(0) = ti_w;
        // X_state.segment<3>(3) = euler;
        X_state_correct.segment<3>(0) = ti_w;
        X_state_correct.segment<3>(3) = euler;

        cout << "--------------------------------------------" << endl;
        Quaterniond qc_w(Rw_c.inverse());
        Vector3d tc_w = -Rw_c.inverse()*tw_c;
        cout << "\ncam in world euler_c_w: \n" << quaternion2euler(qc_w) << "\nimu in world euler_i_w: \n" << euler << endl;
        cout << "\ncam in world tc_w: \n" << tc_w << "\nimu in world ti_w: \n" << ti_w << endl;
        cout << "--------------------------------------------" << endl;

    }
    else
    {
        time_odom_tag_now = msg->header.stamp.toSec();
        //    double t = clock();
        MatrixXd Ct;
        MatrixXd Wt;

        //zt = [p q]
        Vector3d p_temp;
        p_temp(0) = msg->pose.pose.position.x;
        p_temp(1) = msg->pose.pose.position.y;
        p_temp(2) = msg->pose.pose.position.z;
        //四元数转欧拉角  ZYX  roll pitch yaw
        Quaterniond q;
        q.w() = msg->pose.pose.orientation.w;
        q.x() = msg->pose.pose.orientation.x;
        q.y() = msg->pose.pose.orientation.y;
        q.z() = msg->pose.pose.orientation.z;
       
        //Euler transform
        Rw_c = q.toRotationMatrix();
        tw_c = p_temp;
        Ri_w = Rw_c.inverse() * Rc_i.inverse();
        ti_w = -( Rw_c.inverse()*tw_c + Ri_w*tc_i );
        Vector3d euler = mat2euler(Ri_w);  //////////!!!!!!!//////////
        Z_measurement.segment<3>(0) = ti_w;
        Z_measurement.segment<3>(3) = euler;
        cam_system_pub(msg->header.stamp);

        // bool find_proper_frame = search_proper_frame(time_odom_tag_now);
        //find the coordinate time imu
        //not find, use the latest state
        // StateCovariance_correct = StateCovariance_in_rightframe
        // X_state_correct = state_in_rightframe

        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        Kt_kalmanGain = StateCovariance_correct*Ct.transpose() * (Ct*StateCovariance_correct*Ct.transpose() + Wt*Rt*Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
    
        //Prevent innovation changing suddenly when euler from -Pi to Pi
        if(innovation(3) > 6)  innovation(3) -= 2*PI;
        if(innovation(3) < -6) innovation(3) += 2*PI;
        if(innovation(4) > 6)  innovation(4) -= 2*PI;
        if(innovation(4) < -6) innovation(4) += 2*PI;
        if(innovation(5) > 6)  innovation(5) -= 2*PI;
        if(innovation(5) < -6) innovation(5) += 2*PI;
        INNOVATION_ = innovation.segment<3>(3);
        X_state_correct = X_state_correct + Kt_kalmanGain*(innovation);
        StateCovariance_correct = StateCovariance_correct - Kt_kalmanGain*Ct*StateCovariance_correct;
#if 0 //no aligned
        Kt_kalmanGain = StateCovariance*Ct.transpose() * (Ct*StateCovariance*Ct.transpose() + Wt*Rt*Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
    
        //Prevent innovation changing suddenly when euler from -Pi to Pi
        if(innovation(3) > 6)  innovation(3) -= 2*PI;
        if(innovation(3) < -6) innovation(3) += 2*PI;
        if(innovation(4) > 6)  innovation(4) -= 2*PI;
        if(innovation(4) < -6) innovation(4) += 2*PI;
        if(innovation(5) > 6)  innovation(5) -= 2*PI;
        if(innovation(5) < -6) innovation(5) += 2*PI;
        INNOVATION_ = innovation.segment<3>(3);
        X_state += Kt_kalmanGain*(innovation);
        StateCovariance = StateCovariance - Kt_kalmanGain*Ct*StateCovariance;
#endif
// ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
        // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;
        
        // cam_system_pub(msg->header.stamp);
        test_odomtag_call = true;
        odomtag_call = true;
        
        // if(INNOVATION_(0)>6 || INNOVATION_(1)>6 || INNOVATION_(2)>6)
        // cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        if(cnt == 10||cnt==50||cnt==90)
        {
            // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl; 
            // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl; 
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
            // cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        }
        cnt++;
        if(cnt>100) cnt=101;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 4, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s2 = n.subscribe("tag_odom", 4, odom_callback, ros::TransportHints().tcpNoDelay());
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 4);   //freq = imu freq
    cam_odom_pub = n.advertise<nav_msgs::Odometry>("cam_ekf_odom", 4);  

    n.getParam("gyro_cov", gyro_cov);
    n.getParam("acc_cov", acc_cov);
    n.getParam("position_cov", position_cov);
    n.getParam("q_rp_cov", q_rp_cov);
    n.getParam("q_yaw_cov", q_yaw_cov);

    cout << "Q:" << gyro_cov << " " << acc_cov << " R: " << position_cov << " " << q_rp_cov << " " << q_yaw_cov << endl;

    initsys();
    cout << "initsys" << endl;

    cout << "======================" << endl;
    double r = atan2(1,-100);
    double p = asin(-0.707);
    double y = atan2(-1, -100);   
    cout << "r: " << r << " p: " << p << " y: " << y << endl;
    cout << "======================" << endl;

    ros::spin();
}

void system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = "world";
    // odom_fusion.header.frame_id = "imu";
    odom_fusion.pose.pose.position.x = X_state(0);
    odom_fusion.pose.pose.position.y = X_state(1);
    odom_fusion.pose.pose.position.z = X_state(2);
    Quaterniond q;
    q = euler2quaternion(X_state.segment<3>(3));
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    odom_fusion.twist.twist.linear.x = X_state(6);
    odom_fusion.twist.twist.linear.y = X_state(7);
    odom_fusion.twist.twist.linear.z = X_state(8);
    odom_pub.publish(odom_fusion);
}
void cam_system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = "world";
    // odom_fusion.header.frame_id = "imu";
    odom_fusion.pose.pose.position.x = Z_measurement(0);
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;
    q = euler2quaternion(Z_measurement.segment<3>(3));
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    // odom_fusion.twist.twist.angular.x = Z_measurement(3);INNOVATION_
    // odom_fusion.twist.twist.angular.y = Z_measurement(4);
    // odom_fusion.twist.twist.angular.z = Z_measurement(5);
    // odom_fusion.twist.twist.angular.x = INNOVATION_(0);
    // odom_fusion.twist.twist.angular.y = INNOVATION_(1);
    // odom_fusion.twist.twist.angular.z = INNOVATION_(2);
    // Vector3d pp, qq, v, bg, ba;
    // getState(pp, qq, v, bg, ba);
    // odom_fusion.twist.twist.angular.x = ba(0);
    // odom_fusion.twist.twist.angular.y = ba(1);///??????why work??????????//////
    // odom_fusion.twist.twist.angular.z = ba(2);
    odom_fusion.twist.twist.angular.x = diff_time;
    odom_fusion.twist.twist.angular.y = dt;
    cam_odom_pub.publish(odom_fusion);
}

//process model
void initsys()
{
    //set the cam2imu params
    Rc_i = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rc_i << endl;
    tc_i << 0.05, 0.05, 0; 

    //states X [p q pdot bg ba]  [px,py,pz, wx,wy,wz, vx,vy,vz bgx,bgy,bgz bax,bay,baz]
    stateSize = 15;                                                                 // x = [p q pdot bg ba]
    stateSize_pqv = 9;                                                              // x = [p q pdot]
    measurementSize = 6;                                                            // z = [p q]
    inputSize = 6;                                                                  // u = [w a]
    X_state = VectorXd::Zero(stateSize);                                            // x 
    //velocity
    X_state(6) = 0;
    X_state(7) = 0;
    X_state(8) = 0; 
    //bias
    X_state.segment<3>(9) = bg_0;
    X_state.segment<3>(12) = ba_0;
    u_input = VectorXd::Zero(inputSize);
    Z_measurement = VectorXd::Zero(measurementSize);                                // z
    StateCovariance = MatrixXd::Identity(stateSize, stateSize);                     // sigma
    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize);                 // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct
    X_state_correct = X_state;
    StateCovariance_correct = StateCovariance;

    Qt = MatrixXd::Identity(inputSize, inputSize);  //6x6 input [gyro acc]covariance
    Rt = MatrixXd::Identity(measurementSize, measurementSize); //6x6 measurement [p q]covariance

    // You should also tune these parameters
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // //Rt visual odomtry covariance smaller believe measurement more
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
}

void getState(Vector3d& p, Vector3d& q, Vector3d& v, Vector3d& bg, Vector3d& ba)
{
	p = X_state.segment<3>(0);
	q = X_state.segment<3>(3);
	v = X_state.segment<3>(6);
    bg = X_state.segment<3>(9);
    ba = X_state.segment<3>(12);
}

VectorXd F_model(Vector3d gyro, Vector3d acc)
{
    VectorXd f(VectorXd::Zero(stateSize));
    Vector3d p, q, v, bg, ba;
    getState(p, q, v, bg, ba);
    f.segment<3>(0) = v;
    f.segment<3>(3) = w_Body2Euler(q)*(gyro-bg-ng);
    f.segment<3>(6) = gravity + euler2mat(q)*(acc-ba-na);
    f.segment<3>(9) = nbg;
    f.segment<3>(12) = nba;
    return f;
}

VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    g.segment<6>(0) = X_state.segment<6>(0);

    return g;
}

//F_model G_model Jocobian
//diff_f()/diff_x (x_t-1  ut  noise=0)   At     Ft = I+dt*At
MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{
    double cr = cos( q_last(0));
    double sr = sin( q_last(0));
    double cp = cos( q_last(1));
    double sp = sin( q_last(1));
    double cy = cos( q_last(2));
    double sy = sin( q_last(2));
 
    // ng na = 0 nbg nba = 0
    double Ax = acc(0) - ba_last(0);
    double Ay = acc(1) - ba_last(1);
    double Az = acc(2) - ba_last(2);
    // double Wx = gyro(0) - bg_last(0);
    double Wy = gyro(1) - bg_last(1);
    double Wz = gyro(2) - bg_last(2);

    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(stateSize, stateSize));
    MatrixXd diff_f_diff_x_jacobian_pqv(MatrixXd::Zero(stateSize_pqv, stateSize_pqv));

    diff_f_diff_x_jacobian_pqv <<  0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 
      0, 0, 0, 0, 0, 0, 0, 0, 1,  
      0, 0, 0, (sp*(Wy*cr - Wz*sr))/cp, (Wz*cr + Wy*sr)/(cp*cp), 0, 0, 0, 0,  
      0, 0, 0, (- Wz*cr - Wy*sr), 0, 0, 0, 0, 0, 
      0, 0, 0, (Wy*cr - Wz*sr)/cp, (sp*(Wz*cr + Wy*sr))/(cp*cp), 0, 0, 0, 0, 
      0, 0, 0, (Ay*(sr*sy + cr*cy*sp) + Az*(cr*sy - cy*sp*sr)), (Az*cp*cr*cy - Ax*cy*sp + Ay*cp*cy*sr), (Az*(cy*sr - cr*sp*sy) - Ay*(cr*cy + sp*sr*sy) - Ax*cp*sy), 0, 0, 0, 
      0, 0, 0, (- Ay*(cy*sr - cr*sp*sy) - Az*(cr*cy + sp*sr*sy)), (Az*cp*cr*sy - Ax*sp*sy + Ay*cp*sr*sy), (Az*(sr*sy + cr*cy*sp) - Ay*(cr*sy - cy*sp*sr) + Ax*cp*cy), 0, 0, 0,
      0, 0, 0, (Ay*cp*cr - Az*cp*sr), (- Ax*cp - Az*cr*sp - Ay*sp*sr), 0, 0, 0, 0 ;
    
    diff_f_diff_x_jacobian.block<9, 9>(0, 0) = diff_f_diff_x_jacobian_pqv;
    diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -w_Body2Euler(q_last);
    diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -euler2mat(q_last);

    return diff_f_diff_x_jacobian;

    // cp != 0 pitch != 90° !!!!!!!!!
}
//diff_f()/diff_n (x_t-1  ut  noise=0)  Ut    Vt = dt*Ut
MatrixXd diff_f_diff_n(Vector3d q_last)
{
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(stateSize, inputSize));
    diff_f_diff_n_jacobian.block<3,3>(3,0) = -w_Body2Euler(q_last);
    diff_f_diff_n_jacobian.block<3,3>(6,3) = -euler2mat(q_last);

    return diff_f_diff_n_jacobian;
}
//diff_g()/diff_x  (xt~ noise=0)  Ct 
MatrixXd diff_g_diff_x()
{
    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize, stateSize));
    diff_g_diff_x_jacobian.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    diff_g_diff_x_jacobian.block<3,3>(3,3) = MatrixXd::Identity(3,3);

    return diff_g_diff_x_jacobian;
}
//diff_g()/diff_v  (xt~ noise=0) Wt
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize, measurementSize));

    return diff_g_diff_v_jacobian;
}
