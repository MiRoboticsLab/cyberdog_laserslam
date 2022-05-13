/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef LASER_SLAM_LOCAL_SLAM_H_
#define LASER_SLAM_LOCAL_SLAM_H_
#include <memory>
#include <vector>

#include "common/time.h"
#include "common/param.h"
#include "sensor/range_data.h"
#include "sensor/voxel_filter.h"
#include "range_data_matching/map/submap_2d.h"
#include "range_data_matching/map/motion_filter.h"
#include "range_data_matching/scan_matching/ceres_scan_matcher_2d.h"
#include "range_data_matching/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "pose_graph/data_set/trajectory_node.h"
#include "pose_extrapolator/measurement_collector.h"
// Add Range Data
// Local SLAM insertion result {submaps, tracked pose}
// every time laser scan commin, do the scan matching and return the insertion
// result
namespace cartographer {
namespace laser_slam {
struct InsertionResult {
  std::shared_ptr<const pose_graph::optimization::TrajectoryNode::Data>
      node_constant_data;
  std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps;
};

struct MatchingResult {
  common::Time time;
  transform::Rigid3d local_pose;
  sensor::RangeData range_data_in_local;
  std::unique_ptr<const InsertionResult> insertion_result;
};
// To_DO(feixiang Zeng): compatible pose extrapolator without imu
class LocalSlam {
 public:
  explicit LocalSlam(const LocalSlamParam& param)
      : param_(param),
        active_submaps_(nullptr),
        motion_filter_(nullptr),
        pose_extrapolator_(nullptr),
        ceres_scan_matcher_(nullptr),
        real_time_matcher_(nullptr) {
    voxel_filter_param_.max_length = param_.max_length;
    voxel_filter_param_.max_range = param_.max_range_scale;
    voxel_filter_param_.min_num_points = param_.min_num_points;
    active_submaps_.reset(new mapping::ActiveSubmaps2D(param.submap_param));
    MotionFilterParam motion_filter_param;
    motion_filter_param.max_angle_radians = param_.max_angle_radians;
    motion_filter_param.max_distance_meters = param_.max_distance_meters;
    motion_filter_param.max_time_seconds = param_.max_time_seconds;
    motion_filter_.reset(new mapping::MotionFilter(motion_filter_param));
    ceres_scan_matcher_.reset(
        new mapping::scan_matching::CeresScanMatcher2D(param_.ceres_param));
    real_time_matcher_.reset(
        new mapping::scan_matching::RealTimeCorrelativeScanMatcher2D(
            param_.real_time_param));
  }
  virtual ~LocalSlam() {}

  bool InitialExtrapolatorWithPoses(
      const std::vector<transform::TimedRigid3d>& initial_poses);
  // Returns 'MatchingResult' when range data comming,otherwise 'nullptr'.
  std::unique_ptr<MatchingResult> AddRangeData(
      const sensor::PointCloud& timed_point_cloud);

  void AddImuData(const sensor::ImuData& imu_data);

  void AddOdometryData(const sensor::OdometryData& odom_data);

  transform::Rigid3d newest_pose_extrapolator(common::Time time) {
    return pose_extrapolator_->ExtrapolatePose(time);
  }

  void AddPose(const transform::TimedRigid3d& pose) {
    pose_extrapolator_->AddPose(pose);
  }

  common::Time LatestPoseExtrapolatorTime() const {
    common::Time time;
    if (pose_extrapolator_ == nullptr) {
      time = common::Time::min();
    } else {
      time = pose_extrapolator_->LatestExtrapolatorTime();
    }
    return time;
  }

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& range_data,
      const transform::Rigid3d& gravity_alignment);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      const common::Time& time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_point_cloud,
      const transform::Rigid3d& pose_estimated,
      const Eigen::Quaterniond& gravity_alignment);

  std::unique_ptr<transform::Rigid2d> ScanMatch(
      const common::Time& time, const transform::Rigid2d& pose_predicted,
      const sensor::PointCloud& pc);

  sensor::RangeData VoxelFilter(const sensor::RangeData& range_data);

  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;

  LocalSlamParam param_;
  sensor::AdaptiveVoxelFilterParam voxel_filter_param_;

  std::shared_ptr<mapping::ActiveSubmaps2D> active_submaps_;
  std::shared_ptr<mapping::MotionFilter> motion_filter_;

  std::shared_ptr<pose_extrapolator::MeasurementCollector> pose_extrapolator_;
  std::shared_ptr<mapping::scan_matching::CeresScanMatcher2D>
      ceres_scan_matcher_;
  std::shared_ptr<mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>
      real_time_matcher_;

  // accumulated how many laser frame
  int num_accumulated_ = 0;

  sensor::RangeData accumulated_data_;

  std::deque<transform::Rigid2d> pose_queue_;
};
typedef std::shared_ptr<LocalSlam> LocalSlamPtr;
typedef std::shared_ptr<const LocalSlam> LocalSlamConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_LOCAL_SLAM_H_
