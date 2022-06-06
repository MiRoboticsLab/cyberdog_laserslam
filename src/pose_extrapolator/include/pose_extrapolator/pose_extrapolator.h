/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_H_
#define POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_H_
#include <deque>
#include <memory>
#include <mutex>

#include "common/time.h"
#include "pose_extrapolator/pose_extrapolator_interface.h"
#include "pose_extrapolator/filter/complementary_filter.h"
#include "pose_extrapolator/filter/imu_tracker.h"
#include "sensor/imu_data.h"
#include "sensor/odometry_data.h"
#include "transform/rigid_transform.h"

namespace cartographer {
namespace pose_extrapolator {
struct TimedPose {
  common::Time time;
  transform::Rigid3d pose;
};

class PoseExtrapolator : public PoseExtraPolatorInterface {
 public:
  explicit PoseExtrapolator(const FilterParam& param)
      : gravity_time_constant_(param.gravity_constant),
        cached_extrapolated_pose_{common::Time::min(),
                                  transform::Rigid3d::Identity()} {
    pose_queue_duration_ = common::FromSeconds(param.pose_duration_time);
    complementary_filter_.reset(new filter::ComplementaryFilter(param.alpha));
    linear_velocity_from_odometry_.setZero();
    linear_velocity_from_poses_.setZero();
    last_extrapolated_pose_ = transform::TimedRigid3d{
        transform::Rigid3d::Identity(), common::Time::min()};
  }
  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  virtual ~PoseExtrapolator() {}

  static std::unique_ptr<PoseExtrapolator> InitialWithImu(
      const ImuMeasurement& imu_data, const FilterParam& param);

  static std::unique_ptr<PoseExtrapolator> InitialWithPose(
      const std::vector<transform::TimedRigid3d>& initial_poses,
      const FilterParam& param);
  
  void AddPose(const transform::TimedRigid3d& pose) override;

  void AddImuData(const ImuMeasurement& imu_data) override;

  void AddOdometryData(const OdomMeasurement& odom_data) override;

  transform::TimedRigid3d LastAddedPose() override {
    std::lock_guard<std::mutex> lk(last_pose_lk_);
    transform::TimedRigid3d pose = last_extrapolated_pose_;
    return pose;
  }

  // transform::Rigid3d ExtrapolatePose(
  //     common::Time time, const std::deque<ImuMeasurement>& imu_datas,
  //     const std::deque<OdomMeasurement>& odom_datas) override;

  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  common::Time GetLastPoseTime() const override {
    if (timed_pose_queue_.empty()) {
      return common::Time::min();
    }
    return timed_pose_queue_.back().time;
  }

  common::Time GetLatestOdomTime() const override {
    if (odom_data_.empty()) {
      return common::Time::min();
    }
    return odom_data_.back().time;
  }

  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  void TrimImuData();
  void TrimOdomData();
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         filter::ImuTracker* const imu_tracker);

  Eigen::Vector3d ExtrapolateTranslation(common::Time time);
  void AdvanceImuTracker(const common::Time& time,
                         filter::ImuTracker* const imu_tracker) const;

  void UpdateVelocityFromPoses();

  double gravity_time_constant_;
  std::mutex last_pose_lk_;
  common::Duration pose_queue_duration_;
  std::deque<TimedPose> timed_pose_queue_;
  std::deque<ImuMeasurement> imu_data_;
  std::deque<OdomMeasurement> odom_data_;
  filter::ComplementaryFilterPtr complementary_filter_;
  std::unique_ptr<filter::ImuTracker> imu_tracker_;
  std::unique_ptr<filter::ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<filter::ImuTracker> extrapolation_imu_tracker_;
  Eigen::Vector3d angular_velocity_from_odometry_;
  Eigen::Vector3d linear_velocity_from_odometry_;
  Eigen::Vector3d angular_velocity_from_poses_;
  Eigen::Vector3d linear_velocity_from_poses_;
  TimedPose cached_extrapolated_pose_;
  transform::TimedRigid3d last_extrapolated_pose_;
};

}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_POSE_EXTRAPOLATOR_H_
