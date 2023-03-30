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