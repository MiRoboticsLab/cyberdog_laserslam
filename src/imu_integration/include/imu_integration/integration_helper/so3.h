/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeiixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_INTEGRATION_HELPER_SO3_H_
#define IMU_INTEGRATION_INTEGRATION_HELPER_SO3_H_
#include <eigen3/Eigen/Core>

#include "imu_integration/integration_helper/math_util.h"
constexpr double kSmallEps = 1e-10;
namespace imu_integration {
class So3 {
 public:
  So3() { unit_quaternion_.setIdentity(); }
  So3(const So3& other);  // construct from other so3
  explicit So3(
      const Eigen::Matrix3d& rotation);  // construct from rotation matrix
  explicit So3(
      const Eigen::Quaterniond& unit_quaternion);  // construct from quaternion
  So3(double rot_x, double rot_y, double rot_z);   // construct from euler angle
  virtual ~So3() {}
  // Follow paper "IMU Preintegration on Manifold for Efficient Visual-Inertial
  // Maximum-a-posteriori Estimation" Right jocabian and some operation with SO3
  static Eigen::Matrix3d JocabianR(const Eigen::Vector3d& theta) {
    Eigen::Matrix3d j_r = Eigen::Matrix3d::Identity();
    double theta_norm = theta.norm();
    if (theta_norm < 1e-5) {
      return j_r;  // rotation theta is too small, just return identity
    } else {
      Eigen::Vector3d identity_vector = theta.normalized();
      Eigen::Matrix3d k = MathUtil::skew(identity_vector);

      j_r = Eigen::Matrix3d::Identity() -
            (1 - cos(theta_norm)) / theta_norm * k +
            (1 - sin(theta_norm) / theta_norm) * k * k;
    }
    return j_r;
  }
  // inverse of right jocabian
  static Eigen::Matrix3d JocabianRInv(const Eigen::Vector3d& theta) {
    Eigen::Matrix3d j_r_inv = Eigen::Matrix3d::Identity();
    double theta_norm = theta.norm();
    if (theta_norm < 1e-5) {
      return j_r_inv;  // rotation theta is too small, just return identity
    } else {
      Eigen::Vector3d identity_vector = theta.normalized();
      Eigen::Matrix3d k = MathUtil::skew(identity_vector);
      j_r_inv = Eigen::Matrix3d::Identity() + 0.5 * MathUtil::skew(theta) +
                (1.0 - (1.0 + cos(theta_norm)) * theta_norm /
                           (2.0 * sin(theta_norm))) *
                    k * k;
    }
    return j_r_inv;
  }

  void operator=(const So3& so3);

  static Eigen::Matrix3d JocabianL(const Eigen::Vector3d& theta) {
    return JocabianR(-theta);
  }

  static Eigen::Matrix3d JocabianLInv(const Eigen::Vector3d& theta) {
    return JocabianRInv(theta);
  }

  static So3 Exp(const Eigen::Vector3d& omega) {
    double theta;
    return ExpAndTheta(omega, &theta);
  }

  static So3 ExpAndTheta(const Eigen::Vector3d& omega, double* theta);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::Quaterniond unit_quaternion_;
};
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_INTEGRATION_HELPER_SO3_H_