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
#include <algorithm>

#include "pose_extrapolator/pose_extrapolator.h"

namespace cartographer {
namespace pose_extrapolator {
std::unique_ptr<PoseExtrapolator>
PoseExtrapolator::InitialWithImu(const ImuMeasurement &imu_data,
                                 const FilterParam &param) {
    auto extrapolator = std::make_unique<PoseExtrapolator>(param);
    extrapolator->AddImuData(imu_data);
    extrapolator->imu_tracker_ = std::make_unique<filter::ImuTracker>(
        param.gravity_constant, imu_data.time);
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
        imu_data.linear_acceleration);
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
        imu_data.angular_velocity);
    extrapolator->imu_tracker_->Advance(imu_data.time);
    extrapolator->AddPose(transform::TimedRigid3d{
        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()),
        imu_data.time});
    return extrapolator;
}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitialWithPose(
    const std::vector<transform::TimedRigid3d> &initial_poses,
    const FilterParam &param) {
    auto extrapolator = std::make_unique<PoseExtrapolator>(param);
    extrapolator->imu_tracker_ = std::make_unique<filter::ImuTracker>(
        param.gravity_constant, initial_poses[0].pose.rotation(),
        initial_poses[0].timestamp);
    extrapolator->AddPose(initial_poses[0]);
    return extrapolator;
}

Eigen::Quaterniond
PoseExtrapolator::EstimateGravityOrientation(common::Time time) {
    filter::ImuTracker imu_tracker = *imu_tracker_;
    AdvanceImuTracker(time, &imu_tracker);
    return imu_tracker.orientation();
}

void PoseExtrapolator::TrimImuData() {
    while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
           imu_data_[1].time <= timed_pose_queue_.back().time) {
        imu_data_.pop_front();
    }
}

void PoseExtrapolator::TrimOdomData() {
    while (odom_data_.size() > 2 && !timed_pose_queue_.empty() &&
           odom_data_[1].time <= timed_pose_queue_.back().time) {
        odom_data_.pop_front();
    }
}

void PoseExtrapolator::AddImuData(const ImuMeasurement &imu_data) {
    if (timed_pose_queue_.empty()) {
        LOG(INFO) << "stupid";
    }
    CHECK(timed_pose_queue_.empty() ||
          imu_data.time >= timed_pose_queue_.back().time);
    imu_data_.push_back(imu_data);
    TrimImuData();
}

// just for calculate delta pose between two time point
Eigen::Quaterniond
PoseExtrapolator::ExtrapolateRotation(common::Time time,
                                      filter::ImuTracker *const imu_tracker) {
    CHECK_GE(time, imu_tracker->time());
    AdvanceImuTracker(time, imu_tracker);
    const Eigen::Quaterniond old_pose = imu_tracker_->orientation();

    return old_pose.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
    const TimedPose &newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
        common::ToSeconds(time - newest_timed_pose.time);
    LOG_EVERY_N(INFO, 10) << "linear velocity from poses is: "
                          << linear_velocity_from_poses_.transpose()
                          << "linear velocity from odometry is: "
                          << linear_velocity_from_odometry_.transpose()
                          << "delta time is: " << extrapolation_delta;
    if (odom_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

void PoseExtrapolator::AdvanceImuTracker(
    const common::Time &time, filter::ImuTracker *const imu_tracker) const {
    CHECK_GE(time, imu_tracker_->time());
    if (imu_data_.empty() || time < imu_data_.front().time) {
        // Still no Imu Data comming,so just advance ImuTracker use the angular
        // velocity from poses and fake gravity to help stability
        imu_tracker->Advance(time);
        imu_tracker->AddImuAngularVelocityObservation(
            odom_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
        imu_tracker->AddImuLinearAccelerationObservation(
            Eigen::Vector3d::UnitZ());
        return;
    }
    if (imu_tracker->time() < imu_data_.front().time) {
        imu_tracker->Advance(imu_data_.front().time);
    }
    const common::Time tracker_time = imu_tracker->time();
    auto it = std::lower_bound(
        imu_data_.begin(), imu_data_.end(), tracker_time,
        [](const ImuMeasurement &imu_data, const common::Time &time) {
            return imu_data.time < time;
        });
    while (it != imu_data_.end() && it->time < time) {
        imu_tracker->Advance(it->time);
        imu_tracker->AddImuLinearAccelerationObservation(
            it->linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
        ++it;
    }
    imu_tracker->Advance(time);
}

void PoseExtrapolator::AddOdometryData(const OdomMeasurement &odom_data) {
    if (timed_pose_queue_.empty()) {
        LOG(INFO) << "stupid";
    }
    CHECK(timed_pose_queue_.empty() ||
          odom_data.time >= timed_pose_queue_.back().time);
    odom_data_.push_back(odom_data);
    TrimOdomData();
    if (odom_data_.size() < 2) {
        return;
    }

    const OdomMeasurement odometry_oldest_data = odom_data_.front();
    const OdomMeasurement odometry_newest_data = odom_data_.back();
    if (odometry_newest_data.time == odometry_oldest_data.time) {
        LOG(ERROR) << "ODOMETRY DATA PROBLEM";
        return;
    }
    if (odometry_imu_tracker_->time() > odometry_newest_data.time) {
        LOG(ERROR) << "Odom time lower than last extrapolator time";
        return;
    }
    const double odometry_time_delta = common::ToSeconds(
        odometry_oldest_data.time - odometry_newest_data.time);
    const transform::Rigid3d delta_pose =
        odometry_newest_data.pose.inverse() * odometry_oldest_data.pose;
    angular_velocity_from_odometry_ =
        transform::RotationQuaternionToAngleAxisVector(delta_pose.rotation()) /
        odometry_time_delta;
    if (timed_pose_queue_.empty())
        return;
    const Eigen::Vector3d linear_velocity_within_newest_odom_frame =
        delta_pose.translation() / odometry_time_delta;
    const Eigen::Quaterniond orientation_in_newest_odom_frame =
        timed_pose_queue_.back().pose.rotation() *
        ExtrapolateRotation(odometry_newest_data.time,
                            odometry_imu_tracker_.get());
    linear_velocity_from_odometry_ = orientation_in_newest_odom_frame *
                                     linear_velocity_within_newest_odom_frame;
    LOG_EVERY_N(INFO, 100) << "linear velocity is: "
                           << linear_velocity_from_odometry_.transpose();
}

void PoseExtrapolator::UpdateVelocityFromPoses() {
    if (timed_pose_queue_.size() < 2) {
        return;
        // we need two pose to estimate velocity at least
    }
    CHECK(!timed_pose_queue_.empty());
    const TimedPose &newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    const TimedPose &oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    const double queue_delta = common::ToSeconds(newest_time - oldest_time);
    if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
        LOG(WARNING)
            << "Queue too short for velocity estimation. Queue duration: "
            << queue_delta << " s";
        return;
    }
    const transform::Rigid3d &newest_pose = newest_timed_pose.pose;
    const transform::Rigid3d &oldest_pose = oldest_timed_pose.pose;
    linear_velocity_from_poses_ =
        (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
    angular_velocity_from_poses_ =
        transform::RotationQuaternionToAngleAxisVector(
            oldest_pose.rotation().inverse() * newest_pose.rotation()) /
        queue_delta;
}

void PoseExtrapolator::AddPose(const transform::TimedRigid3d &pose) {
    LOG_IF(INFO, pose.pose.translation().norm() > 100)
        << "add pose is: " << pose.pose;
    {
        std::lock_guard<std::mutex> lk(last_pose_lk_);
        last_extrapolated_pose_ = pose;
    }
    if (imu_tracker_ == nullptr) {
        common::Time tracker_start = pose.timestamp;
        if (!imu_data_.empty()) {
            tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ = std::make_unique<filter::ImuTracker>(
            gravity_time_constant_, tracker_start);
    }
    timed_pose_queue_.push_back(TimedPose{pose.timestamp, pose.pose});
    while (timed_pose_queue_.size() > 2 &&
           timed_pose_queue_[1].time <= pose.timestamp - pose_queue_duration_) {
        timed_pose_queue_.pop_front();
    }
    UpdateVelocityFromPoses();
    AdvanceImuTracker(pose.timestamp, imu_tracker_.get());
    TrimOdomData();
    TrimImuData();
    odometry_imu_tracker_ = std::make_unique<filter::ImuTracker>(*imu_tracker_);
    extrapolation_imu_tracker_ =
        std::make_unique<filter::ImuTracker>(*imu_tracker_);
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(common::Time time) {
    const TimedPose &newest_timed_pose = timed_pose_queue_.back();
    CHECK_GE(time, newest_timed_pose.time)
        << "time problem" << time.time_since_epoch().count() << " , "
        << newest_timed_pose.time.time_since_epoch().count();
    if (cached_extrapolated_pose_.time != time) {
        const Eigen::Vector3d translation =
            ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
        const Eigen::Quaterniond rotation =
            newest_timed_pose.pose.rotation() *
            ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        cached_extrapolated_pose_ =
            TimedPose{time, transform::Rigid3d(translation, rotation)};
    }
    return cached_extrapolated_pose_.pose;
}
} // namespace pose_extrapolator
} // namespace cartographer
