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
#include <memory>

#include "laser_slam/back_end.hpp"

namespace cartographer
{
namespace laser_slam
{
BackEnd::BackEnd(int trajectory_id, const BackEndParam & param)
: trajectory_id_(trajectory_id),
  param_(param),
  thread_pool_(param.thread_num_pool)
{
  local_slam_.reset(new LocalSlam(param_.local_slam_param));
  pose_graph_.reset(
    new pose_graph::optimization::PoseGraph2D(
      param_.pose_graph_param,
      std::make_unique<pose_graph::optimization::OptimizationProblem2D>(
        param_.pose_graph_param.optimization_param),
      &thread_pool_));
}
void BackEnd::AddImuData(const sensor::ImuData & imu_data)
{
  local_slam_->AddImuData(imu_data);
}
void BackEnd::AddOdometryData(const sensor::OdometryData & odom_data)
{
  local_slam_->AddOdometryData(odom_data);
}

std::unique_ptr<TmpResult> BackEnd::AddRangeData(
  const sensor::PointCloud & timed_point_cloud)
{
  std::unique_ptr<MatchingResult> local_result =
    local_slam_->AddRangeData(timed_point_cloud);
  if (local_result == nullptr) {return nullptr;}
  std::unique_ptr<sensor::RangeData> range_data;
  std::unique_ptr<TmpResult> tmp_result;
  if (local_result->insertion_result != nullptr) {
    std::shared_ptr<const pose_graph::optimization::TrajectoryNode::Data>
    node_data = local_result->insertion_result->node_constant_data;
    range_data =
      std::make_unique<sensor::RangeData>(local_result->range_data_in_local);
    TmpResult result;
    result.local_pose = local_result->local_pose;
    result.local_range_data = local_result->range_data_in_local;
    tmp_result = std::make_unique<TmpResult>(result);
    // tmp_result->local_pose = local_result->local_pose;
    // tmp_result->local_range_data = local_result->range_data_in_local;
    pose_graph_->GetTrajectoryStates();
    mapping::NodeId node_id =
      pose_graph_->AddNode(
      node_data, trajectory_id_,
      local_result->insertion_result->insertion_submaps);
    return tmp_result;
  }
  return nullptr;
}
}  // namespace laser_slam
}  // namespace cartographer
