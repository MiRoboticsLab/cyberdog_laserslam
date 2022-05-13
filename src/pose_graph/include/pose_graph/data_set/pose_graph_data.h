/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixing@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_DATA_SET_POSE_GRAPH_DATA_H_
#define POSE_GRAPH_DATA_SET_POSE_GRAPH_DATA_H_
#include <map>
#include <set>
#include <vector>

// #include "pose_graph/optimization_problem/optimization_problem_2d.h"
#include "pose_graph/data_set/trajectory_node.h"
#include "range_data_matching/map/submaps.h"
namespace cartographer {
namespace pose_graph {
namespace optimization {
using Submap = mapping::Submap;
using NodeId = mapping::NodeId;
using SubmapId = mapping::SubmapId;

struct NodeSpec2D {
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
  Eigen::Quaterniond gravity_alignment;
};

struct SubmapSpec2D {
  transform::Rigid2d global_pose;
};

struct OdometryDelta2D {
  int trajectory_id;
  mapping::NodeId node_i;
  mapping::NodeId node_j;
  transform::Rigid3d delta_pose;
};
// The current state of the submap in the background threads. After this
// transitions to 'kFinished', all nodes are tried to match
// against this submap. Likewise, all new nodes are matched against submaps in
// that state.
enum class SubmapState { kNoConstraintSearch, kFinished };

enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

// A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
// pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
// 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.

struct Pose {
  transform::Rigid3d zbar_ij;
  double translation_weight;
  double rotation_weight;
};

struct Constraint {
  mapping::SubmapId submap_id;  // 'i' in the paper.
  mapping::NodeId node_id;      // 'j' in the paper.

  // Pose of the node 'j' relative to submap 'i'.
  Pose pose;

  // Differentiates between intra-submap (where node 'j' was inserted into
  // submap 'i') and inter-submap constraints (where node 'j' was not inserted
  // into submap 'i').
  enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
};

struct SubmapPose {
  int version;
  transform::Rigid3d pose;
};

struct SubmapData {
  std::shared_ptr<const mapping::Submap> submap;
  transform::Rigid3d pose;
};

struct InternalSubmapData {
  std::shared_ptr<const Submap> submap;
  SubmapState state = SubmapState::kNoConstraintSearch;

  // IDs of the nodes that were inserted into this map together with
  // constraints for them. They are not to be matched again when this submap
  // becomes 'kFinished'.
  std::set<NodeId> node_ids;
};

struct InitialTrajectoryPose {
  int to_trajectory_id;
  transform::Rigid3d relative_pose;
  common::Time time;
};

struct InternalTrajectoryState {
  enum class DeletionState {
    NORMAL,
    SCHEDULED_FOR_DELETION,
    WAIT_FOR_DELETION
  };

  TrajectoryState state = TrajectoryState::ACTIVE;
  DeletionState deletion_state = DeletionState::NORMAL;
};

struct PoseGraphData {
  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  mapping::MapById<SubmapId, InternalSubmapData> submap_data;

  // Global submap poses currently used for displaying data.
  mapping::MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_2d;

  // Data that are currently being shown.
  mapping::MapById<NodeId, TrajectoryNode> trajectory_nodes;

  //   // Global landmark poses with all observations.
  //   std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
  //       landmark_nodes;

  //   // How our various trajectories are related.
  //   TrajectoryConnectivityState trajectory_connectivity_state;
  int num_trajectory_nodes = 0;
  std::map<int, InternalTrajectoryState> trajectories_state;

  // Set of all initial trajectory poses.
  std::map<int, InitialTrajectoryPose> initial_trajectory_poses;

  std::vector<Constraint> constraints;
};

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_DATA_SET_POSE_GRAPH_DATA_H_
