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
#include "pose_extrapolator/integrator/imu_integration_midpoint.h"

namespace cartographer {
namespace pose_extrapolator {
namespace integrator {
void ImuIntegrationMidPoint::Integrate(
    double dt, const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
    const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
    const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
    const Eigen::Vector3d& delta_v, const Eigen::Vector3d& linearized_ba,
    const Eigen::Vector3d& linearized_bg, Eigen::Vector3d* result_delta_p,
    Eigen::Quaterniond* result_delta_q, Eigen::Vector3d* result_delta_v,
    Eigen::Vector3d* result_linearized_ba,
    Eigen::Vector3d* result_linearized_bg, bool update_jacobian) {
  V3D un_acc_0 = delta_q * (acc_0 - linearized_ba) + gravity_;
  LOG(INFO) << "un acc is: " << un_acc_0.transpose() << "dt is: " << dt;
  V3D un_gyro = 0.5 * (gyro_0 + gyro_1) - linearized_bg;
  Q4D dq = Eigen::Quaterniond(1, un_gyro(0) * dt / 2, un_gyro(1) * dt / 2,
                              un_gyro(2) * dt / 2);
  *result_delta_q = (delta_q * dq).normalized();
  V3D un_acc_1 = *result_delta_q * (acc_1 - linearized_ba) + gravity_;
  V3D un_acc = 0.5 * (un_acc_0 + un_acc_1);
  Eigen::Vector3d delta_p_tmp, delta_v_tmp, linearized_ba_tmp,
      linearized_bg_tmp;
  delta_v_tmp = delta_v + un_acc * dt;
  delta_p_tmp = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
  linearized_ba_tmp = linearized_ba;
  linearized_bg_tmp = linearized_bg;
  if (update_jacobian) {
    MXD f_t = MXD::Zero(kStateDim, kStateDim);
    f_t.block<3, 3>(kPosition, kVelocity) = M3D::Identity();
    f_t.block<3, 3>(kVelocity, kRotation) =
        -delta_q.toRotationMatrix() * MathUtil::skew(acc_1 - linearized_ba);
    f_t.block<3, 3>(kVelocity, kAccBias) = -delta_q.toRotationMatrix();
    f_t.block<3, 3>(kRotation, kRotation) =
        -MathUtil::skew(gyro_1 - linearized_bg);
    f_t.block<3, 3>(kRotation, kGyroBias) = -M3D::Identity();
    MXD g_t = MXD::Zero(kStateDim, kNoiseDim);
    g_t.block<3, 3>(kVelocity, 0) = -delta_q.toRotationMatrix();
    g_t.block<3, 3>(kRotation, 3) = -M3D::Identity();
    g_t.block<3, 3>(kAccBias, 6) = M3D::Identity();
    g_t.block<3, 3>(kGyroBias, 9) = M3D::Identity();
    g_t = g_t * dt;

    const MXD I = MXD::Identity(kStateDim, kStateDim);

    F_jocbian_ = I + f_t * dt + 0.5 * f_t * f_t * dt * dt;
    covariance_ = F_jocbian_ * covariance_ * F_jocbian_.transpose() +
                  g_t * noise_ * g_t.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();
  }
  *result_delta_p = delta_p_tmp;
  *result_delta_v = delta_v_tmp;
  *result_linearized_ba = linearized_ba_tmp;
  *result_linearized_bg = linearized_bg_tmp;
}

void ImuIntegrationMidPoint::IntegrateAngleOnly(
    double dt, const Eigen::Vector3d& gyro_0, const Eigen::Vector3d& gyro_1,
    const Eigen::Quaterniond& delta_q, Eigen::Quaterniond* result_delta_q) {
  V3D un_gyro = 0.5 * (gyro_0 + gyro_1);
  Q4D dq = Eigen::Quaterniond(1, un_gyro(0) * dt / 2, un_gyro(1) * dt / 2,
                              un_gyro(2) * dt / 2);
  *result_delta_q = (delta_q * dq).normalized();
}
}  // namespace integrator
}  // namespace pose_extrapolator
}  // namespace cartographer