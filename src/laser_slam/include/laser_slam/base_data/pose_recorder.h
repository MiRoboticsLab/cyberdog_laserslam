/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef LASER_SLAM_BASE_DATA_POSE_RECORDER_H_
#define LASER_SLAM_BASE_DATA_POSE_RECORDER_H_
#include <fstream>
#include <iomanip>
#include <string>

#include <pose_graph/data_set/pose_graph_data.h>

namespace cartographer {
namespace laser_slam {
class PoseRecorder {
 public:
  PoseRecorder(const std::string& file_name);

  virtual ~PoseRecorder();

  bool Write(const mapping::MapById<mapping::NodeId,
                                    pose_graph::optimization::TrajectoryNode>&
                 trajectory_nodes);

  bool Close();

 private:
  std::string file_name_;
  std::ofstream writer_;
};
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_BASE_DATA_POSE_RECORDER_H_
