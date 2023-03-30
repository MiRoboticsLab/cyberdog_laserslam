// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef LASER_SLAM__BASE_DATA__POSE_RECORDER_HPP_
#define LASER_SLAM__BASE_DATA__POSE_RECORDER_HPP_
#include <pose_graph/data_set/pose_graph_data.h>

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

namespace cartographer
{
namespace laser_slam
{
struct RecordPose
{
  int64 timestamp;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d position;
};
class PoseRecorder
{
public:
  explicit PoseRecorder(const std::string & file_name);

  virtual ~PoseRecorder();

  bool Write(
    const mapping::MapById<mapping::NodeId,
    pose_graph::optimization::TrajectoryNode> &
    trajectory_nodes);

  void InputRecordPose(
    int64 timestamp, const Eigen::Quaterniond & rotation,
    const Eigen::Vector3d & position)
  {
    RecordPose record_pose;
    record_pose.timestamp = timestamp;
    record_pose.rotation = rotation;
    record_pose.position = position;
    poses_.push_back(record_pose);
  }

  bool WriteRecordPose();

  bool Close();

private:
  std::string file_name_;
  std::ofstream writer_;
  std::vector<RecordPose> poses_;
};
}   // namespace laser_slam
}   // namespace cartographer

#endif  // LASER_SLAM__BASE_DATA__POSE_RECORDER_HPP_
