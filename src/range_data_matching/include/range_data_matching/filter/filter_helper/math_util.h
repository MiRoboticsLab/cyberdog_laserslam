/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_MATH_UTIL_H_
#define RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_MATH_UTIL_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace cartographer {
namespace mapping {

class MathUtil {
 public:
  MathUtil() {}
  virtual ~MathUtil() {}

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
      const Eigen::MatrixBase<Derived>& q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
        typename Derived::Scalar(0), -q(0), -q(1), q(0),
        typename Derived::Scalar(0);
    return ans;
  }

  static Eigen::Quaterniond axis_2_quat(const Eigen::Vector3d& axis,
                                        double theta) {
    Eigen::Quaterniond q;

    if (theta < 1e-10) {
      q.w() = 1.0;
      q.x() = q.y() = q.z() = 0;
      return q;
    }

    double magnitude = sin(theta / 2.0);

    q.w() = cos(theta / 2.0);
    q.x() = axis(0) * magnitude;
    q.y() = axis(1) * magnitude;
    q.z() = axis(2) * magnitude;

    return q;
  }

  static Eigen::Quaterniond axis_2_quat(const Eigen::Vector3d& vec) {
    Eigen::Quaterniond q;
    double theta = vec.norm();
    if (theta < 1e-10) {
      q.w() = 1.0;
      q.x() = q.y() = q.z() = 0.0;
      return q;
    }
    Eigen::Vector3d tmp = vec / theta;
    return axis_2_quat(tmp, theta);
  }

  static Eigen::Vector3d r_2_rpy(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;
    rpy(1) = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    rpy(0) = atan2(R(2, 1) / cos(rpy(1)), R(2, 2) / cos(rpy(1)));  // roll
    rpy(2) = atan2(R(1, 0) / cos(rpy(1)), R(0, 0) / cos(rpy(1)));  // yaw
    return rpy;
  }

  static Eigen::Vector3d qua_2_rpy(const Eigen::Quaterniond& q) {
    return r_2_rpy(q.toRotationMatrix());
  }
  static Eigen::Vector3d quat_2_axis(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angular;
    angular(0, 0) = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                               1 - 2 * (q.x() * q.x() - q.y() * q.y()));
    angular(1, 0) = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
    angular(2, 0) = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                               1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return angular;
  }

  static Eigen::Quaterniond rpy_2_qua(const Eigen::Vector3d& rpy) {
    double alpha = rpy(0);
    double belta = rpy(1);
    double gama = rpy(2);
    Eigen::Quaterniond result;
    result.w() = cos(alpha / 2.0) * cos(belta / 2.0) * cos(gama / 2.0) +
                 sin(alpha / 2.0) * sin(belta / 2.0) * sin(gama / 2.0);
    result.x() = sin(alpha / 2.0) * cos(belta / 2.0) * cos(gama / 2.0) -
                 cos(alpha / 2.0) * sin(belta / 2.0) * sin(gama / 2.0);
    result.y() = cos(alpha / 2.0) * sin(belta / 2.0) * cos(gama / 2.0) +
                 sin(alpha / 2.0) * cos(belta / 2.0) * sin(gama / 2.0);
    result.z() = cos(alpha / 2.0) * cos(belta / 2.0) * sin(gama / 2.0) -
                 sin(alpha / 2.0) * sin(belta / 2.0) * cos(gama / 2.0);
    return result;
  }

  static Eigen::Quaterniond qua_plus_angular(const Eigen::Quaterniond& q,
                                             const Eigen::Vector3d& angular) {
    Eigen::Quaterniond dq = rpy_2_qua(angular);
    Eigen::Quaterniond result;
    result = (q * dq).normalized();
    return result;
  }

};  // class MathUtil
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_MATH_UTIL_H_