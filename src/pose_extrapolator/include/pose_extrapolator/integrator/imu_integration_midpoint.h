/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_MIDPOINT_H_
#define POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_MIDPOINT_H_
#include "pose_extrapolator/integrator/imu_integration_interface.h"
#include "pose_extrapolator/filter_helper/math_util.h"

namespace cartographer {
namespace pose_extrapolator {
namespace integrator {
class ImuIntegrationMidPoint : public ImuIntegrationInterface {
 public:
  ImuIntegrationMidPoint(const MXD& init_covariance, const MXD& init_noise,
                         const V3D& gravity)
      : ImuIntegrationInterface(init_covariance, init_noise, gravity) {}
  virtual ~ImuIntegrationMidPoint() {}

  void Integrate(
      double dt, const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
      const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
      const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
      const Eigen::Vector3d& delta_v, const Eigen::Vector3d& linearized_ba,
      const Eigen::Vector3d& linearized_bg, Eigen::Vector3d* result_delta_p,
      Eigen::Quaterniond* result_delta_q, Eigen::Vector3d* result_delta_v,
      Eigen::Vector3d* result_linearized_ba,
      Eigen::Vector3d* result_linearized_bg, bool update_jacobian) override;

  void IntegrateAngleOnly(double dt, const Eigen::Vector3d& gyro_0,
                          const Eigen::Vector3d& gyro_1,
                          const Eigen::Quaterniond& delta_q,
                          Eigen::Quaterniond* result_delta_q) override;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::shared_ptr<const ImuIntegrationMidPoint>
    ImuIntegrationMidPointConstPtr;
typedef std::shared_ptr<ImuIntegrationMidPoint> ImuIntegrationMidPointPtr;
}  // namespace integrator
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_MIDPOINT_H_