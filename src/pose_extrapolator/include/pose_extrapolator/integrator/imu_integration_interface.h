/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_INTERFACE_H_
#define POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_INTERFACE_H_
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>

#include "pose_extrapolator/filter_helper/global_state.h"
namespace cartographer {
namespace pose_extrapolator {
namespace integrator {
class ImuIntegrationInterface {
 public:
  ImuIntegrationInterface(const MXD& init_covariance, const MXD& init_noise,
                          const V3D& gravity)
      : covariance_(init_covariance), noise_(init_noise), gravity_(gravity) {}
  virtual ~ImuIntegrationInterface() {}

  ImuIntegrationInterface() = delete;
  ImuIntegrationInterface(const ImuIntegrationInterface&) = delete;
  /**
   * Follow the paper "Quaternion kinematics for the error-state Kalman filter"
   * @param: acc_0, acc_1, gyro_0, gyro_1, delta_p, delta_q, delta_v,
   * linearized_ba, linearized_bg
   * @output: result_delta_p, result_delta_q, result_delta_v,
   * result_linearized_ba, result_linearized_bg
   */
  virtual void Integrate(
      double dt, const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
      const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
      const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
      const Eigen::Vector3d& delta_v, const Eigen::Vector3d& linearized_ba,
      const Eigen::Vector3d& linearized_bg, Eigen::Vector3d* result_delta_p,
      Eigen::Quaterniond* result_delta_q, Eigen::Vector3d* result_delta_v,
      Eigen::Vector3d* result_linearized_ba,
      Eigen::Vector3d* result_linearized_bg, bool update_jacobian) = 0;

  virtual void IntegrateAngleOnly(double dt, const Eigen::Vector3d& gyro_0,
                                  const Eigen::Vector3d& gyro_1,
                                  const Eigen::Quaterniond& delta_q,
                                  Eigen::Quaterniond* result_delta_q) = 0;

  const MXD f_jocbian() const { return F_jocbian_; }
  MXD& f_jocbian() { return F_jocbian_; }

  const MXD covariance() const { return covariance_; }
  MXD& covariance() { return covariance_; }

  const MXD noise() const { return noise_; }
  MXD& noise() { return noise_; }

  void UpdateCovariance(const MXD& covariance) { covariance_ = covariance; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  MXD F_jocbian_;  // state jocabian transition matrix
  MXD covariance_;
  MXD noise_;
  V3D gravity_;
};  // class ImuIntegrationInterface
typedef std::shared_ptr<const ImuIntegrationInterface>
    ImuIntergrationInterfaceConstPtr;
typedef std::shared_ptr<ImuIntegrationInterface> ImuIntegrationInterfacePtr;
}  // namespace integrator
}  // namespace pose_extrapolator
}  // namespace cartographer
#endif  // POSE_EXTRAPOLATOR_INTEGRATOR_IMU_INTEGRATION_INTERFACE_H_
