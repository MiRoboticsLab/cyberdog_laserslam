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
#ifndef RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_POSE_EXTRAPOLATOR_INTERFACE_H_
#define RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_POSE_EXTRAPOLATOR_INTERFACE_H_
#include "range_data_matching/filter/filter_helper/imu_odom_measurement_assembly.h"
#include "transform/rigid_transform.h"
#include "sensor/point_cloud.h"
#include "range_data_matching/filter/filter.h"

namespace cartographer {
namespace mapping {
struct Params {
  bool use_imu;
  bool use_odometry_update_ekf;
  double gravity_constant;
  double timed_pose_queue_duration;
  FilterParam filter_param;
};
// extrapolator is a singleton for every local slam
// no copy and no copy construct
class PoseExtraPolatorInterface {
 public:
  PoseExtraPolatorInterface(const PoseExtraPolatorInterface&) = delete;
  PoseExtraPolatorInterface& operator=(const PoseExtraPolatorInterface&) =
      delete;
  virtual ~PoseExtraPolatorInterface() {}
  // Init Pose extrapolator by a given pose or imu data
  static std::unique_ptr<PoseExtraPolatorInterface> CreateWithImuData(
      const Params& param, const std::vector<ImuMeasurement>& imu_datas,
      const std::vector<transform::TimedRigid3d>& initial_poses);
  virtual void AddPose(const transform::TimedRigid3d& pose);
  virtual void AddImuData(const ImuMeasurement& imu_data) = 0;
  virtual void AddOdometryData(const OdomMeasurement& odom_data) = 0;
  virtual transform::Rigid3d ExtrapolatePose(double time) = 0;

 protected:
  PoseExtraPolatorInterface() {}
};
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_FILTER_FILTER_HELPER_POSE_EXTRAPOLATOR_INTERFACE_H_
