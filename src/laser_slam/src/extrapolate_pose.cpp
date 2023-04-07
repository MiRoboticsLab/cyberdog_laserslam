// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights
// reserved.
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
#include "laser_slam/extrapolate_pose.hpp"

namespace cartographer {
namespace laser_slam {
void ExtrapolatePose::AddPose(const transform::TimedRigid3d &pose) {
    {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        newest_pose_ = pose;
    }

    poses_.push_back(pose);
    while (poses_.size() > 2 &&
           poses_[1].timestamp <= pose.timestamp - common::FromSeconds(0.01)) {
        poses_.pop_front();
    }
    auto newest_pose = poses_.back();
    auto oldest_pose = poses_.front();
    const double queue_delta =
        common::ToSeconds(newest_pose.timestamp - oldest_pose.timestamp);
    if (queue_delta < 0.01) {
        LOG(INFO) << "Queue too short";
        return;
    }
    linear_velocity_ =
        (newest_pose.pose.translation() - oldest_pose.pose.translation()) /
        queue_delta;
    LOG(INFO) << "Linear velocity is: " << linear_velocity_.transpose();
}

void ExtrapolatePose::AddImuData(const sensor::ImuData &imu_data) {
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        angular_velocity_ = imu_data.angular_velocity;
    }
}

transform::Rigid3d
ExtrapolatePose::ExtrapolateByTime(const common::Time &time) {
    transform::TimedRigid3d newest_pose;
    Eigen::Vector3d angular_velocity;
    {
        std::lock_guard<std::mutex> lk1(imu_mtx_);
        std::lock_guard<std::mutex> lk2(pose_mtx_);
        newest_pose = newest_pose_;
        angular_velocity = angular_velocity_;
    }
    auto imu_datas = imu_datas_;
    auto linear_velocity = linear_velocity_;
    auto common_time = time;
    if (common_time < newest_pose.timestamp) {
        return newest_pose.pose;
    }
    double delta_time = common::ToSeconds(common_time - newest_pose.timestamp);
    Eigen::Quaterniond orientation = newest_pose.pose.rotation();
    auto rotation = transform::AngleAxisVectorToRotationQuaternion(
        Eigen::Vector3d(angular_velocity * delta_time));
    orientation = (orientation * rotation).normalized();
    Eigen::Vector3d translation = newest_pose.pose.translation();
    translation = translation + linear_velocity * delta_time;
    return transform::Rigid3d(translation, orientation);
}

} // namespace laser_slam
} // namespace cartographer