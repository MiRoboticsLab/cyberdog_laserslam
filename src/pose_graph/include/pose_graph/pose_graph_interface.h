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
#ifndef POSE_GRAPH_POSE_GRAPH_INTERFACE_H_
#define POSE_GRAPH_POSE_GRAPH_INTERFACE_H_
#include "pose_graph/optimization_problem/optimization_problem_interface.h"
#include "pose_graph/optimization_problem/optimization_problem_2d.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
template <typename DeltaOdometryData>
class PoseGraph {
 public:
  using NodeId = mapping::NodeId;
  using SubmapId = mapping::SubmapId;
  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;
  PoseGraph() {}
  virtual ~PoseGraph() {}

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  virtual void AddOdometryDeltaData(const DeltaOdometryData& delta_odom) = 0;

  virtual void AddNodeToSubmap(const NodeId& node_id,
                               const SubmapId& submap_id) = 0;

  virtual void RunFinalOptimization() = 0;

  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the states of trajectories.
  virtual std::map<int, TrajectoryState> GetTrajectoryStates() const = 0;

  virtual void SetGlobalSlamOptimizationCallback(
      PoseGraph::GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_POSE_GRAPH_INTERFACE_H_
