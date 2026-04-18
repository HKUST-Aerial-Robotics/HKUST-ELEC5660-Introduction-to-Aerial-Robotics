#ifndef _IMU_VISION_H_
#define _IMU_VISION_H_

#include <Eigen/Eigen>
#include <iostream>

#define Mat(x, y) Eigen::Matrix<double, x, y>
#define Vec(x) Eigen::Matrix<double, x, 1>

namespace ekf_imu_vision {
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 21, 1> Vec21;
typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 6, 6> Mat6x6;
typedef Eigen::Matrix<double, 9, 9> Mat9x9;
typedef Eigen::Matrix<double, 12, 12> Mat12x12;
typedef Eigen::Matrix<double, 15, 15> Mat15x15;
typedef Eigen::Matrix<double, 21, 21> Mat21x21;
typedef Eigen::Matrix<double, 15, 6> Mat15x6;
typedef Eigen::Matrix<double, 6, 15> Mat6x15;
typedef Eigen::Matrix<double, 6, 21> Mat6x21;
typedef Eigen::Matrix<double, 21, 6> Mat21x6;
typedef Eigen::Matrix<double, 9, 15> Mat9x15;
typedef Eigen::Matrix<double, 15, 12> Mat15x12;
typedef Eigen::Matrix<double, 15, 9> Mat15x9;
typedef Eigen::Matrix<double, 3, 15> Mat3x15;
typedef Eigen::Matrix<double, 15, 3> Mat15x3;
typedef Eigen::Matrix<double, 1, 15> Mat1x15;
typedef Eigen::Matrix<double, 15, 1> Mat15x1;

/* f(x, u, n), process model */
Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n);
Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n);
Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n);

// g1(x,v), measurement model of PnP
Vec6 modelG1(const Vec15& x, const Vec6& v);
Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v);
Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v);

// g2(x,v), measurement model of stereo VO relative pose
Vec6 modelG2(const Vec21& x, const Vec6& v);
Mat6x21 jacobiG2x(const Vec21& x, const Vec6& v);
Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v);

}  // namespace ekf_imu_vision

#endif