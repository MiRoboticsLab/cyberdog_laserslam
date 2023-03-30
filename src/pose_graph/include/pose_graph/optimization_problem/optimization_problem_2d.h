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
#ifndef POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_2D_H_
#define POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_2D_H_
#include <vector>
#include <map>
#include <set>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>

#include "common/time.h"
#include "common/param.h"
#include "range_data_matching/map/id.h"
#include "transform/transform.h"
#include "transform/rigid_transform.h"
#include "pose_graph/data_set/pose_graph_data.h"
#include "pose_graph/cost_function/spa_cost_function.h"
#include "pose_graph/optimization_problem/optimization_problem_interface.h"

// Optimization Problem build by CERES, pose graph contain edges and nodes

namespace cartographer {
namespace pose_graph {
namespace optimization {
// struct NodeSpec2D {
//   common::Time time;
//   transform::Rigid2d local_pose_2d;
//   transform::Rigid2d global_pose_2d;
//   Eigen::Quaterniond gravity_alignment;
// };

// struct SubmapSpec2D {
//   transform::Rigid2d global_pose;
// };

// struct OdometryDelta2D {
//   int trajectory_id;
//   mapping::NodeId node_i;
//   mapping::NodeId node_j;
//   transform::Rigid3d delta_pose;
// };

class OptimizationProblem2D
    : public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D,
                                          OdometryDelta2D, transform::Rigid2d> {
 public:
  explicit OptimizationProblem2D(const OptimizationParam& param);
  virtual ~OptimizationProblem2D();

  OptimizationProblem2D(const OptimizationProblem2D&) = delete;
  OptimizationProblem2D& operator=(const OptimizationProblem2D&) = delete;
  const mapping::MapById<mapping::SubmapId, SubmapSpec2D>& submap_data()
      const override {
    return submap_data_;
  }
  const mapping::MapById<mapping::NodeId, NodeSpec2D>& node_data()
      const override {
    return node_data_;
  }
  void InsertTrajectoryNode(const NodeId& node_id,
                            const NodeSpec2D& node_data) override;
  void AddTrajectoryNode(int trajectory_id,
                         const NodeSpec2D& node_data) override;
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid2d& global_submap_pose) override;
  void AddSubMap(int trajectory_id,
                 const transform::Rigid2d& global_submap_pose) override;

  void AddOdometryDeltaData(const OdometryDelta2D& odom) override;

  void Solve(const std::vector<Constraint>& constraints,
             const std::map<int, TrajectoryState>& trajectory_state) override;
  void SetMaxNumIterations(int32_t max_num_iterations) override;

  void TrimSubmap(const SubmapId& submap_id) override;

  void TrimTrajectoryNode(const NodeId& node_id) override;

 private:
  ceres::Solver::Options CreateCeresSolverOptions() {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = param_.use_nonmonotonic_steps;
    options.max_num_iterations = param_.max_num_iterations;
    options.num_threads = param_.num_threads;
    return options;
  }
  OptimizationParam param_;
  mapping::MapById<mapping::NodeId, NodeSpec2D> node_data_;
  mapping::MapById<mapping::SubmapId, SubmapSpec2D> submap_data_;
  std::vector<OdometryDelta2D> odom_data_;
};
}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer
#endif  // POSE_GRAPH_OPTIMIZATION_PROBLEM_OPTIMIZATION_PROBLEM_2D_H_
