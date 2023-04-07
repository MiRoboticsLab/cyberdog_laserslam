// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef LASER_SLAM__EXTRAPOLATE_POSE_HPP_
#define LASER_SLAM__EXTRAPOLATE_POSE_HPP_
#include <deque>
#include <mutex>

#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include "sensor/imu_data.h"

#include "rclcpp/time.hpp"



namespace cartographer 
{
namespace laser_slam
{
 class ExtrapolatePose {
  public:
  ExtrapolatePose() {}
  virtual ~ExtrapolatePose() {}

  void AddPose(const transform::TimedRigid3d& pose);

  void AddImuData(const sensor::ImuData& imu_data);

  transform::Rigid3d ExtrapolateByTime(const common::Time& time);
 
 private:
 std::mutex imu_mtx_;
 std::mutex pose_mtx_;
 std::deque<sensor::ImuData> imu_datas_;
 std::deque<transform::TimedRigid3d> poses_;
 Eigen::Vector3d linear_velocity_;
 Eigen::Vector3d angular_velocity_;
 transform::TimedRigid3d newest_pose_;
 };
}
}






#endif  // LASER_SLAM__EXTRAPOLATE_POSE_HPP_
