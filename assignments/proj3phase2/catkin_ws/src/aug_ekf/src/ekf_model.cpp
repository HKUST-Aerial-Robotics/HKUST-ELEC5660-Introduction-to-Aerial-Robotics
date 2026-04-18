#include <ekf_model.h>

namespace ekf_imu_vision {

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {
  // TODO
  // return the model xdot = f(x,u,n)

  (void)x;
  (void)u;
  (void)n;
  return Vec15::Zero();
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {
  // TODO
  // return the derivative wrt original state df/dx

  (void)x;
  (void)u;
  (void)n;
  return Mat15x15::Zero();
}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {
  // TODO
  // return the derivative wrt noise df/dn

  (void)x;
  (void)u;
  (void)n;
  return Mat15x12::Zero();
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15& x, const Vec6& v) {
  // TODO
  // return the model g(x,v), where x = x_origin

  (void)x;
  (void)v;
  return Vec6::Zero();
}

Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v) {
  // TODO
  // return the derivative wrt original state dz/dx, where x = x_origin

  (void)x;
  (void)v;
  return Mat6x15::Zero();
}

Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v) {
  // TODO;
  // return the derivative wrt noise dz/dv

  (void)x;
  (void)v;
  return Mat6x6::Identity();
}

/* ============================== model of stereo VO relative pose ============================== */

Vec6 modelG2(const Vec21& x, const Vec6& v) {
  // TODO
  // return the model g(x,v), where x = (x_origin, x_augmented)

  (void)x;
  (void)v;
  return Vec6::Zero();
}

Mat6x21 jacobiG2x(const Vec21& x, const Vec6& v) {
  // TODO
  // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)

  (void)x;
  (void)v;
  return Mat6x21::Zero();
}

Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {
  // TODO
  // return the derivative wrt noise dz/dv

  (void)x;
  (void)v;
  return Mat6x6::Identity();
}

}  // namespace ekf_imu_vision
