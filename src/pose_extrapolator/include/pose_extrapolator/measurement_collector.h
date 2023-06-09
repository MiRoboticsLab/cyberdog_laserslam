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
#ifndef POSE_EXTRAPOLATOR_MEASUREMENT_COLLECTOR_H_
#define POSE_EXTRAPOLATOR_MEASUREMENT_COLLECTOR_H_
#include <deque>
#include <mutex>

#include "sensor/imu_data.h"
#include "sensor/odometry_data.h"
#include "pose_extrapolator/pose_extrapolator_interface.h"
#include "pose_extrapolator/pose_extrapolator.h"
#include "transform/rigid_transform.h"

namespace cartographer {
namespace pose_extrapolator {
struct ImuOdomAssemblyData {
  /* data */
};

// collect data by the time extrapolate pose outside
// two deque collect imu and odom data
// trim data every time pose extrapolated, pop the data time before time
// extrapolate pose directly by extrapolator

class MeasurementCollector {
 public:
  explicit MeasurementCollector(const FilterParam& param)
      : param_(param), last_extrapolate_time_(common::Time::min()) {}
  virtual ~MeasurementCollector() {}
  void AddOdometryData(const sensor::OdometryData& odom_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddPose(const transform::TimedRigid3d& pose);
  transform::Rigid3d ExtrapolatePose(const common::Time& time);
  bool InitialWithPose(
      const std::vector<transform::TimedRigid3d>& initial_poses);
  common::Time LatestExtrapolatorTime() const {
    if (pose_extrapolator_ == nullptr) return common::Time::min();
    return pose_extrapolator_->GetLastPoseTime();
  }

  transform::TimedRigid3d LastAddedPose() {
    return pose_extrapolator_->LastAddedPose();
  }

  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) {
    return pose_extrapolator_->EstimateGravityOrientation(time);
  }

 private:
  void TrimImuData(const common::Time& time);
  void TrimOdomData(const common::Time& time);
  std::mutex imu_lk_;
  std::mutex odom_lk_;
  FilterParam param_;
  common::Time last_extrapolate_time_;
  std::deque<sensor::ImuData> imu_datas_;
  std::deque<sensor::OdometryData> odom_datas_;
  std::unique_ptr<PoseExtraPolatorInterface> pose_extrapolator_;
};

}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_MEASUREMENT_COLLECTOR_H_
