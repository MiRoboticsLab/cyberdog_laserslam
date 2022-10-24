/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_graph/optimization_problem/optimization_problem_2d.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
namespace {
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

}  // namespace
OptimizationProblem2D::OptimizationProblem2D(const OptimizationParam& param)
    : param_(param) {}
OptimizationProblem2D::~OptimizationProblem2D() {}

void OptimizationProblem2D::AddTrajectoryNode(int trajectory_id,
                                              const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
}

void OptimizationProblem2D::AddSubMap(
    int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
  LOG(INFO) << submap_data_.SizeOfTrajectoryOrZero(trajectory_id);
}

void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  node_data_.Trim(node_id);
}

// TO_DO(feixiang Zeng): Insert Submap Interface
void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
}

void OptimizationProblem2D::AddOdometryDeltaData(const OdometryDelta2D& odom) {
  odom_data_.push_back(odom);
}

void OptimizationProblem2D::SetMaxNumIterations(
    const int32_t max_num_iterations) {
  param_.max_num_iterations = max_num_iterations;
}

void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, TrajectoryState>& trajectory_state) {
  if (node_data_.empty()) {
    // nothing to optimize
    return;
  }
  std::set<int> frozen_trajectories;
  for (const auto& it : trajectory_state) {
    if (it.second == TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  mapping::MapById<mapping::SubmapId, std::array<double, 3>> C_submaps;
  mapping::MapById<mapping::NodeId, std::array<double, 3>> C_nodes;
  // Start to add parameter block, which indicated as node in pose graph
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    // set first submap as constant in the optimization problem
    if (first_submap || frozen) {
      first_submap = false;
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }

  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }

  // Start to add Constraints, which indicated as edges in pose graph
  // INTER SUBMAP constraint is edge which from loop closure, huber loss was
  // added to ensure optimization problem
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(CreateAutoDiffSpaCostFunction(constraint.pose),
                             constraint.tag == Constraint::INTER_SUBMAP
                                 ? new ceres::HuberLoss(param_.huber_scale)
                                 : nullptr,
                             C_submaps.at(constraint.submap_id).data(),
                             C_nodes.at(constraint.node_id).data());
  }
  // if odometry available, add the constraint between node i j
  if (not odom_data_.empty()) {
    for (auto& odom_constraint : odom_data_) {
      bool frozen =
          frozen_trajectories.count(odom_constraint.trajectory_id) != 0;
      if (not frozen) {
        Pose relative_pose{odom_constraint.delta_pose,
                           param_.odometry_traslation_weight,
                           param_.odometry_rotation_weight};
        problem.AddResidualBlock(CreateAutoDiffSpaCostFunction(relative_pose),
                                 nullptr,
                                 C_nodes.at(odom_constraint.node_i).data(),
                                 C_nodes.at(odom_constraint.node_j).data());
      }
    }
  }

  // Solve
  LOG(INFO) << "optimization";
  ceres::Solver::Summary summary;
  ceres::Solve(CreateCeresSolverOptions(), &problem, &summary);

  if (param_.report_full_summary) {
    LOG(INFO) << summary.FullReport();
  }
  // Store the result
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
    // LOG(INFO) << "submap data is: " << C_submap_id_data.id.trajectory_id
    //           <<
    //           submap_data_.at(C_submap_id_data.id).global_pose.DebugString();
  }

  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
}

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer