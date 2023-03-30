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
#ifndef RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_IMU_BASED_EXTRAPOLATOR_H_
#define RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_IMU_BASED_EXTRAPOLATOR_H_
#include <deque>
#include <mutex>

#include "transform/transform.h"
#include "range_data_matching/filter/imu_integration_interface.h"
#include "range_data_matching/filter/imu_integration_midpoint.h"
#include "range_data_matching/filter/filter.h"
#include "range_data_matching/filter/filter_helper/pose_extrapolator_interface.h"

namespace cartographer {
namespace mapping {
class ImuBasedExtraPolator : public PoseExtraPolatorInterface {
 public:
  explicit ImuBasedExtraPolator(const Params& param)
      : use_imu_(param.use_imu),
        use_odometry_update_ekf_(param.use_odometry_update_ekf),
        gravity_constant_(param.gravity_constant),
        timed_pose_queue_duration_(param.timed_pose_queue_duration),
        last_update_timestamp_(0.0) {
    last_odom_measurement_.SetZero();
  }
  virtual ~ImuBasedExtraPolator() {}

  static std::unique_ptr<ImuBasedExtraPolator> InitialWithImu(
      const Params& param, const std::vector<ImuMeasurement>& imu_data,
      const std::vector<transform::TimedRigid3d>& initialposes);

  void AddPose(const transform::TimedRigid3d& pose) override;

  void AddImuData(const ImuMeasurement& imu_data) override;

  void AddOdometryData(const OdomMeasurement& odom_data) override;

  transform::Rigid3d ExtrapolatePose(double time) override;

 private:
  void TrimImuData(double time);
  void TrimOdometryData(double time);
  void AdvanceEkfSystem(double time);
  transform::Rigid3d IntrapolatePose(double delta_time);
  bool use_imu_;
  bool use_odometry_update_ekf_;
  double gravity_constant_;
  double timed_pose_queue_duration_;
  double last_update_timestamp_;
  std::mutex imu_data_mutex_;
  std::mutex odometry_data_mutex_;
  std::deque<ImuMeasurement> imu_data_;
  std::deque<OdomMeasurement> odom_data_;
  std::deque<transform::TimedRigid3d>
      timed_pose_queue_;  // timed pose queue which cache pose from laser
  FilterPtr filter_;
  OdomMeasurement last_odom_measurement_;
  Eigen::Vector3d linear_velocity_newest_;
  Eigen::Vector3d angular_velocity_newest_;
};
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_IMU_BASED_EXTRAPOLATOR_H_
