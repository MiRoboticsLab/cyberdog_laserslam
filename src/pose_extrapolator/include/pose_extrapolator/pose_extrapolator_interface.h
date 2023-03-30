/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_INTERFACE_H_
#define POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_INTERFACE_H_
#include <deque>

#include "common/param.h"
#include "transform/rigid_transform.h"
#include "sensor/point_cloud.h"
#include "sensor/imu_data.h"
#include "sensor/odometry_data.h"

namespace cartographer {
namespace pose_extrapolator {
// extrapolator is a singleton for every local slam
// no copy and no copy construct
class PoseExtraPolatorInterface {
 public:
  using ImuMeasurement = sensor::ImuData;
  using OdomMeasurement = sensor::OdometryData;
  PoseExtraPolatorInterface(const PoseExtraPolatorInterface&) = delete;
  PoseExtraPolatorInterface& operator=(const PoseExtraPolatorInterface&) =
      delete;
  virtual ~PoseExtraPolatorInterface() {}
  // Init Pose extrapolator by a given pose or imu data
  static std::unique_ptr<PoseExtraPolatorInterface> CreateWithImuData(
      const FilterParam& param, const std::vector<ImuMeasurement>& imu_datas,
      const std::vector<transform::TimedRigid3d>& initial_poses);
  virtual void AddPose(const transform::TimedRigid3d& pose) = 0;
  virtual void AddImuData(const ImuMeasurement& imu_data) = 0;
  virtual void AddOdometryData(const OdomMeasurement& odom_data) = 0;
  virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;

  virtual Eigen::Quaterniond EstimateGravityOrientation(common::Time time) = 0;

  virtual transform::TimedRigid3d LastAddedPose() = 0;
  //   virtual transform::Rigid3d ExtrapolatePose(
  //       common::Time time, const std::deque<ImuMeasurement>& imu_datas,
  //       const std::deque<OdomMeasurement>& odom_dats) = 0;

  virtual common::Time GetLastPoseTime() const = 0;

  virtual common::Time GetLatestOdomTime() const = 0;

 protected:
  PoseExtraPolatorInterface() {}
};
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_INTERFACE_H_
