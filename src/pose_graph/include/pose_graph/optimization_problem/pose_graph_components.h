/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_POSE_GRAPH_OPTIMIZATION_PROBLEM_POSE_GRAPH_COMPONENTS_H_
#define RANGE_DATA_MATCHING_POSE_GRAPH_OPTIMIZATION_PROBLEM_POSE_GRAPH_COMPONENTS_H_
#include <memory>

#include "range_data_matching/map/id.h"
#include "transform/transform.h"
#include "transform/rigid_transform.h"
#include "range_data_matching/map/submaps.h"
#include "pose_graph/data_set/pose_graph_data.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
// // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
// // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
// // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.

// struct Pose {
//   transform::Rigid3d zbar_ij;
//   double translation_weight;
//   double rotation_weight;
// };

// struct Constraint {
//   mapping::SubmapId submap_id;  // 'i' in the paper.
//   mapping::NodeId node_id;      // 'j' in the paper.

//   // Pose of the node 'j' relative to submap 'i'.
//   Pose pose;

//   // Differentiates between intra-submap (where node 'j' was inserted into
//   // submap 'i') and inter-submap constraints (where node 'j' was not inserted
//   // into submap 'i').
//   enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
// };

// // struct OdometryConstraintBetween {
// //   mapping::NodeId node_i;
// //   mapping::NodeId node_j;
// //   // Pose of the node 'j' relative to node 'i'
// //   Pose pose;

// //   int trajectory_id;
// // };
// struct InternalSubmapData {
//   std::shared_ptr<const mapping::Submap> submap;
//   SubmapState state = SubmapState::kNoConstraintSearch;

//   // IDs of the nodes that were inserted into this map together with
//   // constraints for them. They are not to be matched again when this submap
//   // becomes 'kFinished'.
//   std::set<NodeId> node_ids;
// };

// struct SubmapPose {
//   int version;
//   transform::Rigid3d pose;
// };

// struct SubmapData {
//   std::shared_ptr<const mapping::Submap> submap;
//   transform::Rigid3d pose;
// };

// struct InitialTrajectoryPose {
//   int to_trajectory_id;
//   transform::Rigid3d relative_pose;
//   common::Time time;
// };

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_POSE_GRAPH_OPTIMIZATION_PROBLEM_POSE_GRAPH_COMPONENTS_H_
