/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_DATA_SET_TRAJECTORY_NODE_H_
#define POSE_GRAPH_DATA_SET_TRAJECTORY_NODE_H_
#include <memory>
#include <vector>

#include <absl/types/optional.h>
#include <eigen3/Eigen/Core>

#include "visualization/msg/trajectory_node_data.hpp"

#include "common/time.h"
#include "sensor/range_data.h"
#include "transform/rigid_transform.h"
namespace cartographer {
namespace pose_graph {
namespace optimization {
struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  absl::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {
  struct Data {
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaterniond gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;

    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;
  };

  visualization::msg::TrajectoryNodeData ToRosMsg(const Data& node_data) {
    visualization::msg::TrajectoryNodeData data;
    data.header.stamp = common::ToRosTime(node_data.time);
    data.gravity_alignment = transform::ToRos(node_data.gravity_alignment);
    data.filtered_gravity_aligned_point_cloud =
        node_data.filtered_gravity_aligned_point_cloud.ToPointCloud2();
    data.local_pose = transform::ToRos(node_data.local_pose);
    return data;
  }

  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data;

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;
};
}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer
#endif  // POSE_GRAPH_DATA_SET_TRAJECTORY_NODE_H_