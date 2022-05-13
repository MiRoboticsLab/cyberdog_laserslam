/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef LASER_SLAM_BACK_END_H_
#define LASER_SLAM_BACK_END_H_
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "pose_graph/pose_graph_2d.h"
#include "laser_slam/local_slam.h"

/**
 * Backend is optimization of full slam
 * 1. every time local slam result come, Add node is triggered with 'constant
 * data', 'constant_data->local_pose' was determined by scan matching against
 * 'insertion_submaps.front()' and the node data was insert into the
 * 'insertion_submaps'. If 'insertion_submaps.front().finished()' is 'true' data
 * was inserted into this map for the last time.
 * 2. find the 'inter_submap' constraint around the submap pose now on when the
 * determined time reached, region searched determined by parameter of local
 * loop closure.
 * 3. loop closure signal from outside(visual slam), every time the signal from
 * visual will trigger a loop closure.
 * 4. loop closure signal from some landmark detected (eg. charging, QR Code)
 *
 */

namespace cartographer {
namespace laser_slam {
struct TmpResult {
  transform::Rigid3d local_pose;
  sensor::RangeData local_range_data;
};
class BackEnd {
 public:
  BackEnd(int trajectory_id, const BackEndParam& param);
  virtual ~BackEnd() {}

  std::unique_ptr<TmpResult> AddRangeData(
      const sensor::PointCloud& timed_point_cloud);

  void AddOdometryData(const sensor::OdometryData& odom_data);

  void AddImuData(const sensor::ImuData& imu_data);

  common::Time LatestPoseExtrapolatorTime() {
    return local_slam_->LatestPoseExtrapolatorTime();
  }

 private:
  int trajectory_id_;
  BackEndParam param_;
  common::ThreadPool thread_pool_;
  std::shared_ptr<LocalSlam> local_slam_;
  std::shared_ptr<pose_graph::optimization::PoseGraph2D> pose_graph_;
};
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_BACK_END_H_
