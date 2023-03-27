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
#include <string>
#include <vector>
#include <memory>

#include "laser_slam/localization.hpp"
#include "protos/proto_stream.h"

namespace cartographer
{
namespace laser_slam
{
Localization::Localization(const LocalizationParam & param)
: trajectory_id_(0), reloc_id_(0), is_reloc_(false), start_(true),
  state_(State::INITIALIZATION), param_(param), reloc_pose_(nullptr),
  local_slam_(nullptr), thread_pool_(param.thread_num_pool),
  reloc_publisher_thread_(nullptr), pose_graph_(nullptr),
  map_loader_(nullptr)
{
  local_slam_.reset(new LocalSlam(param_.local_slam_param));
}
Localization::~Localization() {}

bool Localization::Stop()
{
  start_ = false;
  while(in_laser_process_) {
    LOG(INFO) << "Laser Still Have Task";
    usleep(1000);
  }
  bool result = pose_graph_->Stop();
  if (reloc_publisher_thread_ && reloc_publisher_thread_->joinable()) {
    reloc_publisher_thread_->join();
  }
  return result;
}

void Localization::SetRelocPublisher(
  const rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
  & publisher)
{
  reloc_publisher_ = publisher;
}

void Localization::SetCallback(const PosePcCallback & callback)
{
  pose_pc_callback_ = callback;
}

bool Localization::Initialize()
{
  param_.pose_graph_param.localization_mode = true;
  pose_graph_.reset(
    new pose_graph::optimization::BundleAdjustment(
      param_.pose_graph_param, &thread_pool_));
  LOG_IF(ERROR, param_.pbstream_file_path.empty()) <<
    "Pbstream Path Wrong, which is: " << param_.pbstream_file_path;
  stream::ProtoStreamReader reader(param_.pbstream_file_path);
  stream::ProtoStreamDeserializer deserializer(&reader);
  map_loader_ = std::make_shared<MapLoader>(deserializer);
  InitialPoseGraph();
  if (!reloc_publisher_thread_) {
    reloc_publisher_thread_.reset(
      new std::thread(
        std::bind(&Localization::RelocPublisherThread, this)));
  } else {
    LOG(WARNING) << "Reloc Publisher Thread Already Exist";
  }
  LOG(INFO) << "Initialize done";
  return true;
}

void Localization::RelocPublisherThread()
{
  while (true) {
    if (!start_) {
      return;
    }
    if (reloc_publisher_ == nullptr) {
      return;
    }
    while (!reloc_publisher_->is_activated()) {
      usleep(1000);
      if (!start_) {
        return;
      }
    }
    std_msgs::msg::Int32 reloc_result;
    if (!pose_graph_->is_reloc()) {
      reloc_result.data = 100;
      reloc_publisher_->publish(reloc_result);

    } else {
      reloc_result.data = 0;
      reloc_publisher_->publish(reloc_result);
      return;
    }
  }
}

void Localization::AddRangeData(const sensor::PointCloud & timed_point_cloud)
{
  if (!inertial_comming_ ||
    timed_point_cloud.time() < local_slam_->LatestPoseExtrapolatorTime())
  {
    LOG(WARNING) <<
      "No Inertial Data or imu data timestamp faster than laser";
    return;
  }
  in_laser_process_ = true;

  // transform::Rigid3d optimized_pose;
  // std::unique_ptr<MatchingResult> matching_result;
  auto matching_result = local_slam_->AddRangeData(timed_point_cloud);
  sensor::RangeData range_data_callback;
  transform::Rigid3d pose_to_cb;
  if (matching_result != nullptr && start_) {
    if (matching_result->insertion_result != nullptr) {
      pose_graph_->AddLocalizationNode(
        matching_result->insertion_result->node_constant_data,
        trajectory_id_,
        matching_result->insertion_result->insertion_submaps);
      LOG(INFO) << "Local To Global is: " <<
        pose_graph_->LocalToGlobalTransform(trajectory_id_)
        .DebugString();
    }
    pose_to_cb = pose_graph_->LocalToGlobalTransform(trajectory_id_) *
      matching_result->local_pose;
    range_data_callback = sensor::TransformRangeData(
      matching_result->range_data_in_local,
      pose_graph_->LocalToGlobalTransform(trajectory_id_).cast<float>());
    if (pose_pc_callback_) {
      range_data_callback.returns.time() = timed_point_cloud.time();
      pose_pc_callback_(pose_to_cb, range_data_callback);
    }
  }
  in_laser_process_ = false;
}

void Localization::AddImuData(const sensor::ImuData & imu_data)
{
  if (local_slam_ == nullptr) {
    return;
  }
  if (!inertial_comming_) {
    inertial_comming_ = true;
  }
  local_slam_->AddImuData(imu_data);
}

void Localization::AddOdometryData(const sensor::OdometryData & odom_data)
{
  if (local_slam_ == nullptr || !inertial_comming_) {
    return;
  }
  local_slam_->AddOdometryData(odom_data);
}

void Localization::GetInitialPose(
  const std::shared_ptr<RelocPose> & reloc_pose)
{
  reloc_pose_ = reloc_pose;
  state_ = State::RELOCATION_SUCESS;
}

void Localization::GetRelocPose(
  const mapping::NodeId & id,
  const RelocPose & pose)
{
  // Get the vision reloc constraint when location normally
  LOG(INFO) << "still not implement";
}

bool Localization::InitialPoseGraph()
{
  // check the reloc pose is nullptr?
  const auto nodes = map_loader_->nodes();
  const auto submaps = map_loader_->submaps();
  const auto constraints = map_loader_->constraints();
  int trajectory_id = map_loader_->trajectory_id();
  pose_graph_->RecoverPoseGraphFromLast(
    trajectory_id, nodes, submaps,
    constraints);
  trajectory_id_ = trajectory_id + 1;
  LOG(INFO) << "pose graph global submap pose of trajectory 0 is: " <<
    pose_graph_->pose_graph_data()
    .global_submap_poses_2d.SizeOfTrajectoryOrZero(
    trajectory_id);
  return true;
}

}  // namespace laser_slam
}  // namespace cartographer
