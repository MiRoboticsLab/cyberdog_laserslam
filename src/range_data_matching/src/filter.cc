// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "range_data_matching/filter/filter.h"

namespace cartographer {
namespace mapping {
void Filter::Predict(const std::vector<ImuMeasurement>& imu_datas) {
  for (size_t i = 0; i < imu_datas.size(); ++i) {
    if (fabs(last_odom_update_time_) < 1e-10) {
      last_odom_update_time_ = imu_datas[i].timestamp;
      last_updated_state_ = state_;
    }
    if (last_imu_data_.acc.isZero() && last_imu_data_.gyro.isZero()) {
      last_imu_data_ = imu_datas[i];
      Q4D init_pose;
      complementary_filter_->GetMeasurement(imu_datas[i].acc, &init_pose);
      state_.rotation_ = init_pose;
      LOG(INFO) << "init pose is: "
                << MathUtil::quat_2_axis(init_pose).transpose();
    } else {
      double dt =
          (imu_datas[i].timestamp - last_imu_data_.timestamp) * kNanoSecToSec;
      V3D result_position;
      V3D result_velocity;
      Q4D result_rotation;
      V3D result_ba;
      V3D result_bg;
      integrator_->Integrate(
          dt, last_imu_data_.acc, last_imu_data_.gyro, imu_datas[i].acc,
          imu_datas[i].gyro, state_.position_, state_.rotation_,
          state_.velocity_, state_.ba_, state_.bg_, &result_position,
          &result_rotation, &result_velocity, &result_ba, &result_bg,
          update_jocabian_);
      Eigen::Quaterniond updated_pose;
      complementary_filter_->Update(result_rotation, imu_datas[i].acc,
                                    &updated_pose);
      V3D velocity =
          (updated_pose * state_.rotation_.inverse()) * state_.velocity_;
      LOG(INFO) << "updated rotation is: "
                << MathUtil::quat_2_axis(updated_pose).transpose()
                << "original is: "
                << MathUtil::quat_2_axis(result_rotation).transpose();
      V3D position =
          state_.position_ + 0.5 * (velocity + state_.velocity_) * dt;
      state_.position_ = position;
      state_.rotation_ = updated_pose;
      state_.velocity_ = velocity;
      state_.ba_ = result_ba;
      state_.bg_ = result_bg;
      last_imu_data_ = imu_datas[i];
    }
  }
}

void Filter::UpdateWithOdom(double dt, const V3D& velocity,
                            const V3D& position) {
  MXD kalman_gain;
  MXD error_state;
  MXD H_Matrix;
  kalman_gain.resize(kStateDim, kMeasurementDim);
  kalman_gain.setZero();
  error_state.resize(kStateDim, 1);
  error_state.setZero();
  H_Matrix.resize(kMeasurementDim, kStateDim);
  H_Matrix.setZero();
  H_Matrix.block<3, 3>(0, 3).setIdentity();
  H_Matrix.block<3, 3>(3, 9).setIdentity();
  H_Matrix.block<3, 3>(6, 12).setIdentity();
  VXD cov = VXD::Zero(kMeasurementDim);
  cov << odom_position_sigma_, odom_position_sigma_, odom_position_sigma_,
      measure_ba_sigma_, measure_ba_sigma_, measure_ba_sigma_,
      measure_bg_sigma_, measure_bg_sigma_, measure_bg_sigma_;
  Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> V = cov.asDiagonal();
  Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> Py =
      H_Matrix * covariance_ * H_Matrix.transpose() + V;
  Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> Py_inv;
  Py_inv.setIdentity();
  Py.llt().solveInPlace(Py_inv);
  kalman_gain = covariance_ * H_Matrix.transpose() * Py_inv;
  Eigen::Matrix<double, kStateDim, kStateDim> I;
  I.setIdentity();
  Eigen::Matrix<double, kStateDim, kStateDim> new_cov;
  new_cov.setIdentity();
  new_cov = (I - kalman_gain * H_Matrix) * covariance_;
  covariance_ = new_cov;
  integrator_->covariance() = covariance_;
  //   V3D position_residual = position - state_.position_;
  V3D velocity_residual = velocity - state_.velocity_;
  V3D ba_residual = ba_measurement_ - state_.ba_;
  V3D bg_residual = bg_measurement_ - state_.bg_;
  Eigen::Matrix<double, kMeasurementDim, 1> residual;
  residual.block<3, 1>(0, 0) = velocity_residual;
  residual.block<3, 1>(3, 0) = ba_residual;
  residual.block<3, 1>(6, 0) = bg_residual;
  error_state = kalman_gain * residual;
  LOG(INFO) << "error is: " << error_state.transpose();
  state_.position_ = state_.position_ + error_state.block<3, 1>(0, 0);
  state_.velocity_ = state_.velocity_ + error_state.block<3, 1>(3, 0);
  state_.rotation_ =
      (state_.rotation_ * MathUtil::axis_2_quat(error_state.block<3, 1>(6, 0)))
          .normalized();
  state_.ba_ = state_.ba_ + error_state.block<3, 1>(9, 0);
  state_.bg_ = state_.bg_ + error_state.block<3, 1>(12, 0);
}

std::vector<ImuOdomAssemblyMeasurement> Filter::FindAssemblyMeasurement(
    double time, const std::vector<ImuMeasurement>& imu_data,
    const std::vector<OdomMeasurement>& odom_data) {
  size_t imu_index = 0;
  std::vector<ImuOdomAssemblyMeasurement> result;
  ImuOdomAssemblyMeasurement assemly_data;
  for (size_t i = 0; i < odom_data.size(); ++i) {
    for (; imu_index < imu_data.size(); ++imu_index) {
      if (imu_data[imu_index].timestamp <= odom_data[i].timestamp &&
          odom_data[i].timestamp <= time) {
        assemly_data.imu_measurements.push_back(imu_data[imu_index]);
      } else {
        break;
      }
    }
    if (odom_data[i].timestamp < time) {
      assemly_data.odom_measurement = odom_data[i];
      result.push_back(assemly_data);
      assemly_data.SetZero();
    } else {
      break;
    }
  }
  if ((imu_index < imu_data.size()) && imu_data[imu_index].timestamp <= time) {
    for (; imu_index < imu_data.size(); ++imu_index) {
      if (imu_data[imu_index].timestamp <= time) {
        assemly_data.imu_measurements.push_back(imu_data[imu_index]);
      }
    }
    result.push_back(assemly_data);
  }
  return result;
}

void Filter::UpdatePositionToNewest(double dt) {
  state_.position_ = state_.position_ + newest_linear_velocity_ * dt;
}

void Filter::Advance(double time, const std::vector<ImuMeasurement>& imu_data,
                     const std::vector<OdomMeasurement>& odom_data) {
  std::vector<ImuOdomAssemblyMeasurement> imu_odoms;
  imu_odoms = FindAssemblyMeasurement(time, imu_data, odom_data);
  LOG(INFO) << "assembly data finded";
  for (size_t i = 0; i < imu_odoms.size(); ++i) {
    if (not imu_odoms[i].imu_measurements.empty()) {
      // if (not(fabs(newest_updated_time_) < 1e-10)) {
      //   double dt = (imu_odoms[i].imu_measurements[0].timestamp -
      //                newest_updated_time_) *
      //               kNanoSecToSec;
      //   LOG(INFO) << "dt is: " << dt;
      //   UpdateVelocityAndAngular(dt);
      // }
      Predict(imu_odoms[i].imu_measurements);
      // covariance_.block<9, 9>(6, 6) =
      //     integrator_->covariance().block<9, 9>(6, 6);
      // LOG(INFO) << "covariance is: " << covariance_.diagonal().transpose();
      covariance_ = integrator_->covariance();
    }
    if (not imu_odoms[i].odom_measurement.IsZero()) {
      LOG(INFO) << imu_odoms[i].odom_measurement.timestamp;
      if (fabs(last_odom_update_time_) < 1e-10) {
        last_odom_update_time_ = imu_odoms[i].odom_measurement.timestamp;
        continue;
      } else {
        // if (not(fabs(newest_updated_time_) < 1e-10)) {
        //   double delta_t =
        //       (imu_odoms[i].odom_measurement.timestamp -
        //       newest_updated_time_) * kNanoSecToSec;
        //   UpdateVelocityAndAngular(delta_t);
        // }
        double dt =
            (imu_odoms[i].odom_measurement.timestamp - last_odom_update_time_) *
            kNanoSecToSec;
        LOG(INFO) << "dt is : " << dt;

        Eigen::Vector3d velocity;
        LOG(INFO) << "velocity is: " << velocity.transpose();
        velocity = state_.rotation_ * imu_odoms[i].odom_measurement.position;

        Eigen::Vector3d position;
        Eigen::Vector3d delta_position;
        delta_position = velocity * dt;
        position = last_updated_state_.position_ + delta_position;
        UpdateWithOdom(dt, velocity, position);
        UpdateNewestLinearVelocity(velocity);
        UpdateNewestAngularVelocity(dt, last_updated_state_.rotation_,
                                    state_.rotation_);
        newest_updated_time_ = imu_odoms[i].odom_measurement.timestamp;
        last_odom_update_time_ = imu_odoms[i].odom_measurement.timestamp;
        last_updated_state_ = state_;
        LOG(INFO) << "updated";
      }
    }
    LOG(INFO) << "state rotation is: "
              << MathUtil::quat_2_axis(state_.rotation_).transpose();
    LOG(INFO) << "state velocity is: " << state_.velocity_.transpose();
    LOG(INFO) << "state position is: " << state_.position_.transpose();
    LOG(INFO) << "state ba is: " << state_.ba_.transpose();
    LOG(INFO) << "state bg is: " << state_.bg_.transpose();
  }
}
}  // namespace mapping
}  // namespace cartographer
