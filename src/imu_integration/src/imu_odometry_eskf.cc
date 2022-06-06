/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <iostream>
#include <iomanip>
#include <fstream>

#include "imu_integration/imu_odometry_eskf.h"

namespace imu_integration {
bool ImuOdometryEskf::Initialization() {
  if (not filter_) return false;
  filter_->Initialization();
  state_.SetIdentity();
  return true;
}

void ImuOdometryEskf::PredictWithImu(double timestamp,
                                     const Eigen::Vector3d& acc,
                                     const Eigen::Vector3d& gyro) {
  if (is_first_) {
    first_imu_time_ = timestamp;
    last_imu_time_ = timestamp;
    last_acc_ = acc;
    last_gyro_ = gyro;
    is_first_ = false;
  } else {
    double dt = (timestamp - last_imu_time_);
    last_imu_time_ = timestamp;
    std::cout << "dt of imu is: " << dt << std::endl;
    std::cout << "imu acc is: " << acc.transpose() << std::endl;
    filter_->Predict(dt, last_acc_, acc, last_gyro_, gyro);
    last_acc_ = acc;
    last_gyro_ = gyro;
  }
}

// update state with odometry data
// injection of error state
void ImuOdometryEskf::UpdateWithOdom(double timestamp,
                                     const Eigen::Vector3d& linear_velocity,
                                     const Eigen::Vector3d& angular_velocity) {
  Eigen::Matrix<double, 15, 6> kalman_gain;
  Eigen::Matrix<double, 15, 1> error_state;
  Eigen::Matrix<double, 6, 15> H_matrix;
  H_matrix.setZero();
  H_matrix.block<3, 3>(3, 6).setIdentity();
  H_matrix.block<3, 3>(0, 3).setIdentity();
  VXD cov = VXD::Zero(6);
  cov << odom_sigma_velocity_, odom_sigma_velocity_, odom_sigma_velocity_,
      odom_sigma_angular_, odom_sigma_angular_, 10 * odom_sigma_angular_;
  std::cout << "odom covariance is: " << cov.transpose() << std::endl;
  Eigen::Matrix<double, 6, 6> V = cov.asDiagonal();
  Eigen::Matrix<double, 6, 6> Py =
      H_matrix * filter_->covariance() * H_matrix.transpose() + V;
  Eigen::Matrix<double, 6, 6> py_inv;
  py_inv.setIdentity();
  Py.llt().solveInPlace(py_inv);
  kalman_gain = filter_->covariance() * H_matrix.transpose() * py_inv;
  Eigen::Matrix<double, 15, 15> I;
  I.setIdentity();
  Eigen::Matrix<double, 15, 15> new_covariance;
  new_covariance.setIdentity();
  new_covariance = (I - kalman_gain * H_matrix) * filter_->covariance();
  std::cout << "covariance is: " << new_covariance.diagonal().transpose()
            << std::endl;
  std::cout << "H-matrix is: " << H_matrix.transpose() << std::endl;
  if (first_update_) {
    double dt = (timestamp - first_imu_time_);
    UpdateState(dt, linear_velocity, angular_velocity, kalman_gain,
                new_covariance);
    last_odom_time_ = timestamp;
    first_update_ = false;
  } else {
    double dt = (timestamp - last_odom_time_);
    UpdateState(dt, linear_velocity, angular_velocity, kalman_gain,
                new_covariance);
    last_odom_time_ = timestamp;
  }
}
// To_DO: transform velocity to global(imu) coordinate
void ImuOdometryEskf::UpdateState(
    double dt, const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity,
    const Eigen::Matrix<double, 15, 6>& kalman_gain,
    const Eigen::Matrix<double, 15, 15>& covariance) {
  // R - P - Y
  V3D odom_angular = angular_velocity;

  Eigen::Matrix<double, 6, 1> residual;
  residual.setZero();
  Eigen::Matrix<double, 6, 1> estimated_state;
  Eigen::Matrix<double, 3, 1> rot;
  rot = MathUtil::qua_2_rpy(filter_->state().rotation_);
  // estimated_state = filter_->state().velocity_;
  estimated_state.block<3, 1>(0, 0) = filter_->state().velocity_;
  estimated_state.block<3, 1>(3, 0) = rot;
  Eigen::Matrix<double, 6, 1> measurement;
  measurement.block<3, 1>(0, 0) =
      filter_->state().rotation_.toRotationMatrix() * linear_velocity;
  // measurement.block<3, 1>(3, 0) = angular_velocity;
  measurement.block<3, 1>(3, 0) = MathUtil::qua_2_rpy(
      MathUtil::qua_plus_angular(state_.rotation_, angular_velocity));
  std::cout << "velocity transformed is: " << measurement.transpose()
            << " with rpy is: "
            << MathUtil::qua_2_rpy(filter_->state().rotation_).transpose()
            << std::endl;
  // measurement = linear_velocity;
  // measurement.block<3, 1>(3, 0) = odom_angular;
  residual = measurement - estimated_state;
  std::cout << "residual is: " << residual.transpose() << std::endl;
  std::cout << "kalman gain is: " << kalman_gain << std::endl;
  Eigen::Matrix<double, 15, 1> error;
  error = kalman_gain * residual;
  std::cout << "error is: " << error.transpose() << std::endl;
  GlobalState state;
  state.position_ = filter_->state().position_ + error.block<3, 1>(0, 0);
  state.velocity_ = filter_->state().velocity_ + error.block<3, 1>(3, 0);
  state.rotation_ = MathUtil::qua_plus_angular(filter_->state().rotation_,
                                               error.block<3, 1>(6, 0));
  // state.rotation_ = filter_->state().rotation_;
  Eigen::Vector3d ba_error;
  ba_error = error.block<3, 1>(9, 0);
  state.ba_ = filter_->state().ba_ + ba_error;
  state.bg_ = filter_->state().bg_ + error.block<3, 1>(12, 0);
  // state.gravity_ = filter_->state().gravity_ + error.block<3, 1>(15, 0);
  filter_->Update(state, covariance);
  std::cout << "before state rotation is: "
            << MathUtil::qua_2_rpy(filter_->state().rotation_).transpose()
            << "error rotation is: " << error.block<3, 1>(6, 0).transpose()
            << "result rotation state is: "
            << MathUtil::qua_2_rpy(state.rotation_).transpose()
            << "bg error is: " << error.block<3, 1>(12, 0).transpose()
            << std::endl;
  std::fstream output1;
  output1.open("/home/zfx/imu_odom.txt", std::ios::app);
  output1 << std::setiosflags(std::ios::fixed) << std::setprecision(11)
          << measurement(0) << " " << measurement(1) << " " << measurement(2)
          << " " << measurement(3) << " " << measurement(4) << " "
          << measurement(5) << " " << estimated_state(0) << " "
          << estimated_state(1) << " " << estimated_state(2) << " "
          << estimated_state(3) << " " << estimated_state(4) << " "
          << estimated_state(5) << std::endl;
  output1.close();
  std::fstream output;
  output.open("/home/zfx/error.txt", std::ios::app);
  output << std::setiosflags(std::ios::fixed) << std::setprecision(11)
         << error(0) << " " << error(1) << " " << error(2) << " " << error(3)
         << " " << error(4) << " " << error(5) << " " << error(6) << " "
         << error(7) << " " << error(8) << " " << error(9) << " " << error(10)
         << " " << error(11) << " " << error(12) << " " << error(13) << " "
         << error(14) << std::endl;
  output.close();
  state_ = state;
  // state_ = filter_->state();
}
}  // namespace imu_integration
