/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_INTERFACE_H_
#define POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_INTERFACE_H_
#include "range_data_matching/map/id.h"
#include "pose_graph/optimization_problem/pose_graph_components.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
template <typename NodeDataType, typename SubmapDataType,
          typename OdometryDataType, typename RigidTransformType>
class OptimizationProblemInterface {
 public:
  using NodeId = mapping::NodeId;
  using SubmapId = mapping::SubmapId;
  OptimizationProblemInterface() {}
  virtual ~OptimizationProblemInterface() {}

  OptimizationProblemInterface(const OptimizationProblemInterface&) = delete;
  OptimizationProblemInterface& operator=(const OptimizationProblemInterface&) =
      delete;

  virtual void AddTrajectoryNode(int trajectory_id,
                                 const NodeDataType& node_data) = 0;
  virtual void AddSubMap(int trajectory_id,
                         const RigidTransformType& global_submap_pose) = 0;

  virtual void AddOdometryDeltaData(const OdometryDataType& odom) = 0;
  virtual void InsertTrajectoryNode(const NodeId& node_id,
                                    const NodeSpec2D& node_data) = 0;
  virtual void InsertSubmap(const SubmapId& submap_id,
                            const transform::Rigid2d& global_submap_pose) = 0;
  virtual void Solve(
      const std::vector<Constraint>& constraints,
      const std::map<int, TrajectoryState>& trajectory_state) = 0;

  virtual const mapping::MapById<NodeId, NodeDataType>& node_data() const = 0;

  virtual const mapping::MapById<SubmapId, SubmapDataType>& submap_data()
      const = 0;

  virtual void SetMaxNumIterations(int32_t max_num_iterations) = 0;

  virtual void TrimSubmap(const SubmapId& submap_id) = 0;

  virtual void TrimTrajectoryNode(const NodeId& node_id) = 0;
};

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_INTERFACE_H_
