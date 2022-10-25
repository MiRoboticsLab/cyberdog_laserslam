/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/local_slam.h"

namespace cartographer {
namespace laser_slam {
void LocalSlam::AddImuData(const sensor::ImuData& imu_data) {
  if (pose_extrapolator_ == nullptr) {
    FilterParam param;
    param.use_filter = param_.use_filter;
    param.gravity_constant = param_.gravity_constant;
    param.pose_duration_time = param_.pose_duration_time;
    pose_extrapolator_.reset(
        new pose_extrapolator::MeasurementCollector(param));
    pose_extrapolator_->AddImuData(imu_data);
    return;
  }
  pose_extrapolator_->AddImuData(imu_data);
}

bool LocalSlam::InitialExtrapolatorWithPoses(
    const std::vector<transform::TimedRigid3d>& initial_poses) {
  FilterParam param;
  param.use_filter = param_.use_filter;
  param.gravity_constant = param_.gravity_constant;
  param.pose_duration_time = param_.pose_duration_time;
  pose_extrapolator_.reset(new pose_extrapolator::MeasurementCollector(param));
  pose_extrapolator_->InitialWithPose(initial_poses);
  return true;
}

void LocalSlam::AddOdometryData(const sensor::OdometryData& odom_data) {
  if (pose_extrapolator_ == nullptr) {
    return;
  }
  pose_extrapolator_->AddOdometryData(odom_data);
}
sensor::RangeData LocalSlam::VoxelFilter(const sensor::RangeData& range_data) {
  return sensor::RangeData{
      range_data.origin,
      sensor::VoxelFilter(range_data.returns, param_.voxel_filter_size),
      sensor::VoxelFilter(range_data.misses, param_.voxel_filter_size)};
}

sensor::RangeData LocalSlam::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData transformed_data = sensor::TransformRangeData(
      range_data, transform_to_gravity_aligned_frame);
  return sensor::RangeData{
      transformed_data.origin,
      sensor::VoxelFilter(transformed_data.returns, param_.voxel_filter_size),
      sensor::VoxelFilter(transformed_data.misses, param_.voxel_filter_size)};
}

// TO_DO(Feixiang Zeng) : calculate pointcloud by the timed poses
std::unique_ptr<MatchingResult> LocalSlam::AddRangeData(
    const sensor::PointCloud& timed_point_cloud) {
  // if use imu data, pose extrapolator should be checked is nullptr or no
  if (pose_extrapolator_ == nullptr) {
    LOG(WARNING) << " pose extrapolator not initialized yet";
    return nullptr;
  }
  // cast point cloud to RangeData which contain returns and miss
  if (num_accumulated_ == 0) {
    accumulated_data_ = sensor::RangeData{{}, {}, {}};
  }
  if (timed_point_cloud.empty()) {
    LOG(WARNING) << "droped pointcloud because of empty";
    return nullptr;
  }

  // the pose extrapolated from imu and odom predicted (in global frame)
  transform::Rigid3d pose =
      pose_extrapolator_->ExtrapolatePose(timed_point_cloud.time());
  const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
      pose_extrapolator_->EstimateGravityOrientation(timed_point_cloud.time()));
  Eigen::Vector3f original = pose.translation().cast<float>();
  for (size_t i = 0; i < timed_point_cloud.size(); ++i) {
    sensor::RangefinderPoint hit_in_local = sensor::RangefinderPoint{
        pose.cast<float>() * timed_point_cloud[i].position};
    const Eigen::Vector3f delta = hit_in_local.position - original;
    const float range = delta.norm();
    if (range >= param_.min_range) {
      if (range <= param_.max_range) {
        accumulated_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            original + param_.missing_data_ray_length / range * delta;
        accumulated_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= param_.num_accumulated) {
    num_accumulated_ = 0;
    accumulated_data_.origin = pose.translation().cast<float>();
    // transform range data to origin(0,0),but orientation is transformed to
    // local frame already
    return AddAccumulatedRangeData(
        timed_point_cloud.time(),
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * pose.inverse().cast<float>(),
            accumulated_data_),
        gravity_alignment);
  }
  return nullptr;
}

std::unique_ptr<transform::Rigid2d> LocalSlam::ScanMatch(
    const common::Time& time, const transform::Rigid2d& pose_predicted,
    const sensor::PointCloud& pc) {
  if (active_submaps_->submaps().empty()) {
    return std::make_unique<transform::Rigid2d>(pose_predicted);
  }
  std::shared_ptr<const mapping::Submap2D> matching_submap =
      active_submaps_->submaps().front();
  transform::Rigid2d initial_ceres_pose = pose_predicted;
  if (param_.use_real_time_correlative_scan_matching) {
    const double score = real_time_matcher_->Match(
        pose_predicted, pc, *matching_submap->grid(), &initial_ceres_pose);
    LOG_IF_EVERY_N(WARNING, score < 0.55, 10)
        << "Real Time Correlative Scan Matching Got : " << score
        << "initial ceres pose is: " << initial_ceres_pose.DebugString();
  }

  auto pose_observation = std::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(pose_predicted.translation(), initial_ceres_pose,
                             pc, *matching_submap->grid(),
                             pose_observation.get(), &summary);
  LOG_EVERY_N(INFO, 100)
      << "residual translation is: "
      << (pose_observation->translation() - pose_predicted.translation()).norm()
      << " angle residual is: "
      << (pose_observation->rotation().angle() -
          pose_predicted.rotation().angle());

  return pose_observation;
}

std::unique_ptr<MatchingResult> LocalSlam::AddAccumulatedRangeData(
    common::Time time, const sensor::RangeData& range_data,
    const transform::Rigid3d& gravity_alignment) {
  // check range data returns is empty or not, if empty, drop it and return
  // nullptr
  if (range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data";
    return nullptr;
  }
  const transform::Rigid3d non_gravity_aligned_pose =
      pose_extrapolator_->ExtrapolatePose(time);
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose * gravity_alignment.inverse());

  // Voxel filter adaptively
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(range_data.returns, voxel_filter_param_);

  if (filtered_gravity_aligned_point_cloud.empty()) {
    LOG(WARNING) << "empty returns";
    return nullptr;
  }
  // prediction pose will transform range data to local frame again(only
  // translation)
  std::unique_ptr<transform::Rigid2d> pose_estimated_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);

  if (pose_estimated_2d == nullptr) {
    LOG(WARNING) << "scan matching failed";
    return nullptr;
  }

  const transform::Rigid3d pose_estimated =
      transform::Embed3D(*pose_estimated_2d) * gravity_alignment;

  pose_extrapolator_->AddPose(transform::TimedRigid3d{pose_estimated, time});

  sensor::RangeData range_data_in_local = sensor::TransformRangeData(
      range_data, transform::Embed3D(pose_estimated_2d->cast<float>()));

  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimated, gravity_alignment.rotation());
  return std::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimated, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

std::unique_ptr<InsertionResult> LocalSlam::InsertIntoSubmap(
    const common::Time& time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_point_cloud,
    const transform::Rigid3d& pose_estimated,
    const Eigen::Quaterniond& gravity_alignment) {
  // motion filter judge insert into map or not, if no, return nullptr
  if (motion_filter_->IsSimilar(time, pose_estimated)) {
    return nullptr;
  }
  std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps =
      active_submaps_->InsertRangeData(range_data_in_local);

  // return the insertion result
  return std::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const pose_graph::optimization::TrajectoryNode::Data>(
          pose_graph::optimization::TrajectoryNode::Data{
              time, gravity_alignment, filtered_point_cloud, pose_estimated}),
      std::move(insertion_submaps)});
}
}  // namespace laser_slam
}  // namespace cartographer
