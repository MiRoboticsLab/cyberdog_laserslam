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
#ifndef POSE_EXTRAPOLATOR_FILTER_TRACKER_H_
#define POSE_EXTRAPOLATOR_FILTER_TRACKER_H_
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <deque>

#include <glog/logging.h>

#include "pose_extrapolator/filter_helper/global_state.h"
#include "pose_extrapolator/integrator/imu_integration_interface.h"
#include "pose_extrapolator/integrator/imu_integration_midpoint.h"
#include "pose_extrapolator/filter/complementary_filter.h"
#include "common/param.h"
#include "common/time.h"
#include "transform/transform.h"
#include "sensor/imu_data.h"
#include "sensor/odometry_data.h"

constexpr double kGravityConstant = 9.81;

namespace cartographer {
namespace pose_extrapolator {
namespace filter {
class Tracker {
 public:
  using ImuMeasurement = sensor::ImuData;
  using OdomMeasurement = sensor::OdometryData;
  explicit Tracker(const FilterParam& param);

  virtual ~Tracker() {}

  void AddImuData(const ImuMeasurement& imu_data);

  void AddOdometryData(const OdomMeasurement& odom_data);

  void AddPose(const transform::TimedRigid3d& pose);

  transform::Rigid3d Advance(const common::Time& time);

 private:
  transform::Rigid3d TrackPose(const common::Time& time,
                               std::deque<ImuMeasurement>& imu_datas,
                               std::deque<OdomMeasurement>& odom_datas);
  transform::Rigid3d EkfSystem(const common::Time& time,
                               std::deque<ImuMeasurement>& imu_datas,
                               std::deque<OdomMeasurement>& odom_datas);
  void TrimImuData(const common::Time& time);
  void TrimOdometryData(const common::Time& time);
  void Reset();
  std::mutex odom_lk_;
  std::mutex imu_lk_;
  std::mutex pose_lk_;
  bool update_jocabian_;
  bool first_advance_;
  bool first_imu_;
  double odom_position_sigma_;
  double odom_angular_sigma_;
  double measure_ba_sigma_;
  double measure_bg_sigma_;
  common::Time newest_time_;
  common::Time front_time_;
  transform::TimedRigid3d pose_updated_;
  ImuMeasurement last_imu_meas_;
  OdomMeasurement last_odom_meas_;
  GlobalState state_;
  GlobalState last_updated_state_;
  Eigen::Vector3d newest_velocity_;
  Eigen::Vector3d newest_angular_velocity_from_imu_;
  std::deque<ImuMeasurement> imu_datas_;
  std::deque<OdomMeasurement> odom_datas_;
  integrator::ImuIntegrationInterfacePtr integrator_;
  MXD covariance_;
  filter::ComplementaryFilterPtr complementary_filter_;
};

typedef std::shared_ptr<Tracker> TrackerPtr;
typedef std::shared_ptr<const Tracker> TrackerConstPtr;

}  // namespace filter
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_FILTER_TRACKER_H_
