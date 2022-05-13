/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <iostream>

#include "imu_integration/error_state_kalman_filter.h"

namespace imu_integration {
// TO_DO(feixiang zeng): use acc estimate initial roll pitch
bool ErrorStateKalmanFilter::Initialization() {
  if (integration_type_ == 0) {
    integrator_.reset(new ImuIntegrationMidPointEskf(use_acc_));
  } else {
    return false;
  }
  integrator_->SetGravity(parameters_.gravity);
  MXD covariance;
  covariance.resize(kStateDim, kStateDim);
  covariance.setZero();
  covariance.block<3, 3>(kPosition, kPosition) =
      V3D(parameters_.position_sigma, parameters_.position_sigma,
          parameters_.position_sigma)
          .asDiagonal();  // position
  covariance.block<3, 3>(kVelocity, kVelocity) =
      V3D(parameters_.velocity_sigma, parameters_.velocity_sigma,
          parameters_.velocity_sigma)
          .asDiagonal();  // velocity
  covariance.block<3, 3>(kRotation, kRotation) =
      V3D(parameters_.theta_sigma, parameters_.theta_sigma,
          parameters_.theta_sigma)
          .asDiagonal();  // theta
  covariance.block<3, 3>(kAccBias, kAccBias) =
      V3D(parameters_.ba_sigma, parameters_.ba_sigma, parameters_.ba_sigma)
          .asDiagonal();  // ba
  covariance.block<3, 3>(kGyroBias, kGyroBias) =
      V3D(parameters_.bg_sigma, parameters_.bg_sigma, parameters_.bg_sigma)
          .asDiagonal();  // bg
                          //   covariance.block<3, 3>(kGra, kGra) =
  //       V3D(parameters_.gravity_sigma, parameters_.gravity_sigma,
  //           parameters_.gravity_sigma)
  //           .asDiagonal();  // gravity
  integrator_->SetInitCovariance(covariance);
  MXD noise;
  noise.resize(kNoiseDim, kNoiseDim);
  noise.setZero();
  noise.block<3, 3>(0, 0) =
      V3D(parameters_.vi_sigma, parameters_.vi_sigma, parameters_.vi_sigma)
          .asDiagonal();
  noise.block<3, 3>(3, 3) =
      V3D(parameters_.wi_sigma, parameters_.wi_sigma, parameters_.wi_sigma)
          .asDiagonal();
  noise.block<3, 3>(6, 6) =
      V3D(parameters_.thetai_sigma, parameters_.thetai_sigma,
          parameters_.thetai_sigma)
          .asDiagonal();
  noise.block<3, 3>(9, 9) =
      V3D(parameters_.ai_sigma, parameters_.ai_sigma, parameters_.ai_sigma)
          .asDiagonal();
  integrator_->SetNoise(noise);
  state_.SetIdentity();
  return true;
}

void ErrorStateKalmanFilter::Predict(double dt, const V3D& acc_0,
                                     const V3D& acc_1, const V3D& gyro_0,
                                     const V3D& gyro_1) {
  Eigen::Quaterniond result_q;
  V3D result_v;
  V3D result_p;
  V3D result_linearized_ba;
  V3D result_linearized_bg;
  std::cout << "start integration" << std::endl;
  integrator_->Integrate(dt, acc_0, gyro_0, acc_1, gyro_1, state_.position_,
                         state_.rotation_, state_.velocity_, state_.ba_,
                         state_.bg_, &result_p, &result_q, &result_v,
                         &result_linearized_ba, &result_linearized_bg, true);
  state_.position_ = result_p;
  state_.velocity_ = result_v;
  state_.rotation_ = result_q;
  state_.ba_ = result_linearized_ba;
  state_.bg_ = result_linearized_bg;
  std::cout << "integration rpy is : "
            << MathUtil::qua_2_rpy(state_.rotation_).transpose() << std::endl;
}
}  // namespace imu_integration