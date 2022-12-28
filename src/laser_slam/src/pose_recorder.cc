/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/base_data/pose_recorder.h"

namespace cartographer {
namespace laser_slam {
PoseRecorder::PoseRecorder(const std::string& file_name)
    : file_name_(file_name) {}
PoseRecorder::~PoseRecorder() {}

bool PoseRecorder::Write(
    const mapping::MapById<mapping::NodeId,
                           pose_graph::optimization::TrajectoryNode>&
        trajectory_nodes) {
  writer_.open(file_name_);
  writer_ << std::fixed;
  for (const auto& node : trajectory_nodes) {
    double time = common::ToRosTime(node.data.time()).seconds();
    int64 timestamp = common::ToUniversal(node.data.time());
    auto translation = node.data.global_pose.translation();
    auto rotation = node.data.global_pose.rotation();
    writer_ << std::setprecision(10) << time << " " << translation.x() << " "
            << translation.y() << " " << translation.z() << " " << rotation.x()
            << " " << rotation.y() << " " << rotation.z() << " " << rotation.w()
            << std::endl;
  }
  return true;
}

bool PoseRecorder::WriteRecordPose() {
  if (poses_.empty()) return false;
  writer_.open(file_name_);
  writer_ << std::fixed;
  for (auto pose : poses_) {
    writer_ << std::setprecision(10) << pose.timestamp << " "
            << pose.position.x() << " " << pose.position.y() << " "
            << pose.position.z() << " " << pose.rotation.x() << " "
            << pose.rotation.y() << " " << pose.rotation.z() << " "
            << pose.rotation.w() << std::endl;
  }
  return true;
}

bool PoseRecorder::Close() { writer_.close(); }

}  // namespace laser_slam
}  // namespace cartographer
