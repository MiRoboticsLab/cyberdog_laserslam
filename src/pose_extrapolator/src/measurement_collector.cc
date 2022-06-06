/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_extrapolator/measurement_collector.h"

namespace cartographer {
namespace pose_extrapolator {
void MeasurementCollector::AddImuData(const sensor::ImuData& imu_data) {
  {
    std::lock_guard<std::mutex> lk(imu_lk_);
    if (pose_extrapolator_ == nullptr) {
      std::vector<sensor::ImuData> imu_datas;
      imu_datas.push_back(imu_data);
      std::vector<transform::TimedRigid3d> pose_datas;
      pose_datas.push_back(transform::TimedRigid3d{
          transform::Rigid3d::Rotation(Eigen::Quaterniond::Identity()),
          imu_data.time});
      pose_extrapolator_ = PoseExtraPolatorInterface::CreateWithImuData(
          param_, imu_datas, pose_datas);
      last_extrapolate_time_ = imu_data.time;
      return;
    }
    imu_datas_.push_back(imu_data);
  }
}

bool MeasurementCollector::InitialWithPose(
    const std::vector<transform::TimedRigid3d>& initial_poses) {
  CHECK(pose_extrapolator_ == nullptr);
  std::vector<sensor::ImuData> fake_imu_datas;
  fake_imu_datas.push_back(sensor::ImuData{common::Time::min(),
                                           Eigen::Vector3d(0.0, 0.0, 10.0),
                                           Eigen::Vector3d(0.0, 0.0, 0.0)});
  pose_extrapolator_ = PoseExtraPolatorInterface::CreateWithImuData(
      param_, fake_imu_datas, initial_poses);
  LOG(INFO) << "extrapolator initial success";
  return true;
}

void MeasurementCollector::AddOdometryData(
    const sensor::OdometryData& odom_data) {
  if (pose_extrapolator_ == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(odom_lk_);
    odom_datas_.push_back(odom_data);
  }
}

void MeasurementCollector::AddPose(const transform::TimedRigid3d& pose) {
  if (pose_extrapolator_ == nullptr) {
    LOG(INFO) << "return cause of nullptr";
    return;
  }
  pose_extrapolator_->AddPose(pose);
}

void MeasurementCollector::TrimImuData(const common::Time& time) {
  while (not imu_datas_.empty() && imu_datas_.front().time < time) {
    imu_datas_.pop_front();
  }
}

void MeasurementCollector::TrimOdomData(const common::Time& time) {
  while (not odom_datas_.empty() && odom_datas_.front().time < time) {
    odom_datas_.pop_front();
  }
}
// Get data between last pose extrapolated time and time now
// send data into extrapolator
transform::Rigid3d MeasurementCollector::ExtrapolatePose(
    const common::Time& time) {
  std::deque<sensor::OdometryData> odom_data;
  std::deque<sensor::ImuData> imu_data;
  {
    std::lock_guard<std::mutex> lk_imu(imu_lk_);
    std::lock_guard<std::mutex> lk_odom(odom_lk_);
    odom_data = odom_datas_;
    imu_data = imu_datas_;
    TrimOdomData(time);
    TrimImuData(time);
  }
  transform::Rigid3d result;
  if (pose_extrapolator_ != nullptr) {
    if (not odom_data.empty()) {
      auto odom_it = std::lower_bound(
          odom_data.begin(), odom_data.end(), last_extrapolate_time_,
          [](const sensor::OdometryData& odom, const common::Time& time) {
            return odom.time < time;
          });

      while (odom_it != odom_data.end() && odom_it->time < time) {
        pose_extrapolator_->AddOdometryData(*odom_it);
        ++odom_it;
      }
    }
    if (not imu_data.empty()) {
      auto imu_it = std::lower_bound(
          imu_data.begin(), imu_data.end(), last_extrapolate_time_,
          [](const sensor::ImuData& imu, const common::Time& time) {
            return imu.time < time;
          });

      while (imu_it != imu_data.end() && imu_it->time < time) {
        pose_extrapolator_->AddImuData(*imu_it);
        ++imu_it;
      }
    }
    result = pose_extrapolator_->ExtrapolatePose(time);
    last_extrapolate_time_ = time;
  } else {
    result = transform::Rigid3d::Identity();
    LOG(INFO) << "return a identity";
  }
  return result;
}

}  // namespace pose_extrapolator
}  // namespace cartographer