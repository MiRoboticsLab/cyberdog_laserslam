/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_ERROR_STATE_KALMAN_FILTER_H_
#define IMU_INTEGRATION_ERROR_STATE_KALMAN_FILTER_H_

#include "imu_integration/imu_integration_interface.h"
#include "imu_integration/imu_integration_midpoint_eskf.h"
#include "imu_integration/integration_helper/global_state.h"

namespace imu_integration {
struct EskfParameters {
  double position_sigma;
  double velocity_sigma;
  double theta_sigma;
  double ba_sigma;
  double bg_sigma;
  double gravity_sigma;
  double vi_sigma;      // whith noise v
  double thetai_sigma;  // whith noise theta
  double ai_sigma;      // whith noise acc
  double wi_sigma;      // whith noise gyro
  V3D gravity;
};

class ErrorStateKalmanFilter {
 public:
  ErrorStateKalmanFilter(double position_sigma, double velocity_sigma,
                         double theta_sigma, double ba_sigma, double bg_sigma,
                         double gravity_sigma, double vi_sigma,
                         double thetai_sigma, double ai_sigma, double wi_sigma,
                         bool update_jocabian, bool use_acc,
                         int integration_type, const V3D& gravity)
      : update_jocabian_(update_jocabian),
        use_acc_(use_acc),
        integration_type_(integration_type) {
    parameters_.position_sigma = position_sigma;
    parameters_.velocity_sigma = velocity_sigma;
    parameters_.theta_sigma = theta_sigma;
    parameters_.ba_sigma = ba_sigma;
    parameters_.bg_sigma = bg_sigma;
    parameters_.gravity_sigma = gravity_sigma;
    parameters_.vi_sigma = vi_sigma;
    parameters_.wi_sigma = wi_sigma;
    parameters_.thetai_sigma = thetai_sigma;
    parameters_.ai_sigma = ai_sigma;
    parameters_.gravity = gravity;
  }
  virtual ~ErrorStateKalmanFilter() {}

  bool Initialization();

  void Predict(double dt, const V3D& acc_0, const V3D& acc_1, const V3D& gyro_0,
               const V3D& gyro_1);
  // measurement update, calculate H jocabian first
  // H is measurement fuction h(xt)'s jocbian respect to error state delta xt
  // and then is alpha h/alpha delta x = alpha h/ xt * xt / delta xt according
  // to chain rules, xt/delta xt could be calculated, what we need is alpha h/xt
  // named HX
  void Update(const GlobalState& state,
              const Eigen::Matrix<double, kStateDim, kStateDim>& covariance) {
    state_ = state;
    std::cout << "updated state velocity is: " << state.velocity_.transpose()
              << std::endl;
    integrator_->covariance() = covariance;
  }

  // GlobalState& state() {
  //   std::cout << "x integration rpy is : "
  //             << MathUtil::qua_2_rpy(state_.rotation_).transpose();
  //   return state_;
  // }
  const GlobalState state() const { return state_; }

  void UpdateCovariance(const Eigen::Matrix<double, 18, 18>& covariance) {
    integrator_->UpdateCovariance(covariance);
  }

  MXD& covariance() { return integrator_->covariance(); }

  const MXD covariance() const { return integrator_->covariance(); }

 private:
  int integration_type_;
  bool update_jocabian_;
  bool use_acc_;
  EskfParameters parameters_;
  GlobalState state_;
  ImuIntegrationInterfacePtr integrator_;
};
typedef std::shared_ptr<ErrorStateKalmanFilter> ErrorStateKalmanFilterPtr;
typedef std::shared_ptr<const ErrorStateKalmanFilter>
    ErrorStateKalmanFilterConstPtr;
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_ERROR_STATE_KALMAN_FILTER_H_
