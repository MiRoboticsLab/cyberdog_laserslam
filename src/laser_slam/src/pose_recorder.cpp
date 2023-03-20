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

#include "laser_slam/base_data/pose_recorder.hpp"

namespace cartographer
{
namespace laser_slam
{
PoseRecorder::PoseRecorder(const std::string & file_name)
: file_name_(file_name) {}
PoseRecorder::~PoseRecorder() {}

bool PoseRecorder::Write(
  const mapping::MapById<mapping::NodeId,
  pose_graph::optimization::TrajectoryNode> &
  trajectory_nodes)
{
  writer_.open(file_name_);
  writer_ << std::fixed;
  for (const auto & node : trajectory_nodes) {
    double time = common::ToRosTime(node.data.time()).seconds();
    int64 timestamp = common::ToUniversal(node.data.time());
    auto translation = node.data.global_pose.translation();
    auto rotation = node.data.global_pose.rotation();
    writer_ << std::setprecision(10) << time << " " << translation.x() << " " <<
      translation.y() << " " << translation.z() << " " << rotation.x() <<
      " " << rotation.y() << " " << rotation.z() << " " << rotation.w() <<
      std::endl;
  }
  return true;
}

bool PoseRecorder::WriteRecordPose()
{
  if (poses_.empty()) {return false;}
  writer_.open(file_name_);
  writer_ << std::fixed;
  for (auto pose : poses_) {
    writer_ << std::setprecision(10) << pose.timestamp << " " <<
      pose.position.x() << " " << pose.position.y() << " " <<
      pose.position.z() << " " << pose.rotation.x() << " " <<
      pose.rotation.y() << " " << pose.rotation.z() << " " <<
      pose.rotation.w() << std::endl;
  }
  return true;
}

bool PoseRecorder::Close() {writer_.close();}

}  // namespace laser_slam
}  // namespace cartographer
