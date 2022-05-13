/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "imu_integration/integration_helper/so3.h"

namespace imu_integration {
So3::So3(const So3& other) : unit_quaternion_(other.unit_quaternion_) {
  unit_quaternion_.normalized();
}
So3::So3(const Eigen::Matrix3d& rotation) : unit_quaternion_(rotation) {
  unit_quaternion_.normalized();
}
So3::So3(const Eigen::Quaterniond& unit_quaternion)
    : unit_quaternion_(unit_quaternion) {
  assert(unit_quaternion_.squaredNorm() > kSmallEps);
  unit_quaternion_.normalized();
}
So3::So3(double rot_x, double rot_y, double rot_z) {}

void So3::operator=(const So3& other) {
  this->unit_quaternion_ = other.unit_quaternion_;
}

So3 ExpAndTheta(const Eigen::Vector3d& omega, double* theta) {
  *theta = omega.norm();
  double half_theta = 0.5 * (*theta);

  double imag_factor;                    // imagine factor
  double real_factor = cos(half_theta);  // real factor
  if ((*theta) < kSmallEps) {
    double theta_sq = (*theta) * (*theta);
    double theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / (*theta);
  }
  So3 result(Eigen::Quaterniond(real_factor, imag_factor * omega.x(),
                                imag_factor * omega.y(),
                                imag_factor * omega.z()));

  return result;
}
}  // namespace imu_integration
