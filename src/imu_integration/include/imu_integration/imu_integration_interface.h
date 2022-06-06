/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_IMU_INTEGRATION_INTERFACE_H_
#define IMU_INTEGRATION_IMU_INTEGRATION_INTERFACE_H_
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "imu_integration/integration_helper/global_state.h"
namespace imu_integration {
class ImuIntegrationInterface {
 public:
  ImuIntegrationInterface(bool use_acc) : use_acc_(use_acc) {
    init_covariance_ = false;
  }
  virtual ~ImuIntegrationInterface() {}
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

  const MXD f_jocbian() const { return F_jocbian_; }
  MXD& f_jocbian() { return F_jocbian_; }

  const MXD covariance() const { return covariance_; }
  MXD& covariance() { return covariance_; }

  const MXD noise() const { return noise_; }
  MXD& noise() { return noise_; }

  void SetInitCovariance(const MXD& covariance) {
    if (not init_covariance_) {
      covariance_ = covariance;
      std::cout << "covariance is: " << covariance.diagonal().transpose()
                << std::endl;
      init_covariance_ = true;
    } else {
      return;
    }
  }

  void SetNoise(const MXD& noise) { noise_ = noise; }

  void SetGravity(const V3D& gravity) { gravity_ = gravity; }

  void UpdateCovariance(const MXD& covariance) { covariance_ = covariance; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool use_acc_;
  bool init_covariance_;
  MXD F_jocbian_;  // state jocabian transition matrix
  MXD covariance_;
  MXD noise_;
  V3D gravity_;
};  // class ImuIntegrationInterface
typedef std::shared_ptr<const ImuIntegrationInterface>
    ImuIntergrationInterfaceConstPtr;
typedef std::shared_ptr<ImuIntegrationInterface> ImuIntegrationInterfacePtr;
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_IMU_INTEGRATION_INTERFACE_H_
