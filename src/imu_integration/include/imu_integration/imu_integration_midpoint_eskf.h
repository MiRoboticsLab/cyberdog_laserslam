/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_IMU_INTEGRATION_MIDPOINT_ESKF_H_
#define IMU_INTEGRATION_IMU_INTEGRATION_MIDPOINT_ESKF_H_
#include "imu_integration/imu_integration_interface.h"
#include "imu_integration/integration_helper/math_util.h"

namespace imu_integration {
class ImuIntegrationMidPointEskf : public ImuIntegrationInterface {
 public:
  ImuIntegrationMidPointEskf(bool use_acc) : ImuIntegrationInterface(use_acc) {
    F_jocbian_.resize(kStateDim, kStateDim);
    covariance_.resize(kStateDim, kStateDim);
    noise_.resize(kNoiseDim, kNoiseDim);
    F_jocbian_.setIdentity();
    noise_.setIdentity();
    gravity_ << 0.0, -kGravity, 0.0;
  }
  virtual ~ImuIntegrationMidPointEskf() {}
  // Integration with midpoint, different discrete method correspond to
  // different equations and then transition matrix will be different
  void Integrate(
      double dt, const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
      const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
      const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
      const Eigen::Vector3d& delta_v, const Eigen::Vector3d& linearized_ba,
      const Eigen::Vector3d& linearized_bg, Eigen::Vector3d* result_delta_p,
      Eigen::Quaterniond* result_delta_q, Eigen::Vector3d* result_delta_v,
      Eigen::Vector3d* result_linearized_ba,
      Eigen::Vector3d* result_linearized_bg, bool update_jacobian) override;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};  // class ImuIntegrationMidPointEskf
typedef std::shared_ptr<const ImuIntegrationMidPointEskf>
    ImuIntegrationMidPointEskfConstPtr;
typedef std::shared_ptr<ImuIntegrationMidPointEskf>
    ImuIntegrationMidPointEskfPtr;
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_IMU_INTEGRATION_MIDPOINT_ESKF_H_
