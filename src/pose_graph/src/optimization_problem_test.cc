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
#include <gtest/gtest.h>
#include <glog/logging.h>

#include "pose_graph/optimization_problem/optimization_problem_2d.h"

TEST(OPTIMIZATION, OPT) {
  cartographer::OptimizationParam param = {1e1, 1e5, 1e5, true, 50, 7, false};
  cartographer::pose_graph::optimization::OptimizationProblem2D optimization(
      param);
  cartographer::pose_graph::optimization::NodeSpec2D node1, node2, node3, node4,
      node5, node6, node7, node8;
  cartographer::pose_graph::optimization::SubmapSpec2D submap1, submap2,
      submap3;
  std::vector<cartographer::pose_graph::optimization::Constraint> constraints;
  submap1.global_pose =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.0, 0.0), 0.0);
  submap2.global_pose =
      cartographer::transform::Rigid2d(Eigen::Vector2d(1.5, 0.0), 0.0);
  submap3.global_pose =
      cartographer::transform::Rigid2d(Eigen::Vector2d(3.0, 0.0), 0.0);
  node1.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.0, 0.0), 0.0);
  node1.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.0, 0.0), 0.0);
  node2.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.5, 0.0), 0.0);
  node2.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.5, 0.0), 0.0);
  node3.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.8, 0.0), 0.0);
  node3.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.8, 0.0), 0.0);
  node4.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(1.6, 0.0), 0.0);
  node4.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.1, 0.0), 0.0);
  node5.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(1.9, 0.0), 0.0);
  node5.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.4, 0.0), 0.0);
  node6.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(2.3, 0.0), 0.0);
  node6.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.8, 0.0), 0.0);
  node7.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(3.1, 0.0), 0.0);
  node7.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.1, 0.0), 0.0);
  node8.global_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(3.8, 0.0), 0.0);
  node8.local_pose_2d =
      cartographer::transform::Rigid2d(Eigen::Vector2d(0.8, 0.0), 0.0);
  optimization.AddSubMap(0, submap1.global_pose);
  optimization.AddSubMap(0, submap2.global_pose);
  optimization.AddSubMap(0, submap3.global_pose);
  optimization.AddTrajectoryNode(0, node1);
  optimization.AddTrajectoryNode(0, node2);
  optimization.AddTrajectoryNode(0, node3);
  optimization.AddTrajectoryNode(0, node4);
  optimization.AddTrajectoryNode(0, node5);
  optimization.AddTrajectoryNode(0, node6);
  optimization.AddTrajectoryNode(0, node7);
  optimization.AddTrajectoryNode(0, node8);
  for (auto submap_id_data : optimization.submap_data()) {
    LOG(INFO) << "submap data is: " << submap_id_data.id
              << " data is: " << submap_id_data.data.global_pose.DebugString();
  }
  for (auto node_id_data : optimization.node_data()) {
    LOG(INFO) << "node data id: " << node_id_data.id
              << " data is: " << node_id_data.data.global_pose_2d.DebugString();
  }
  cartographer::transform::Rigid3d relate1(
      Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate2(
      Eigen::Vector3d(0.5, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate3(
      Eigen::Vector3d(0.8, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate4(
      Eigen::Vector3d(1.53, 0, 0), Eigen::Quaterniond::Identity());  // inter
  cartographer::transform::Rigid3d relate5(
      Eigen::Vector3d(0.1, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate6(
      Eigen::Vector3d(1.86, 0, 0), Eigen::Quaterniond::Identity());  // inter
  cartographer::transform::Rigid3d relate7(
      Eigen::Vector3d(0.42, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate8(
      Eigen::Vector3d(2.28, 0, 0), Eigen::Quaterniond::Identity());  // inter
  cartographer::transform::Rigid3d relate9(
      Eigen::Vector3d(0.81, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate10(
      Eigen::Vector3d(1.605, 0, 0),
      Eigen::Quaterniond::Identity());  // inter to submap2
  cartographer::transform::Rigid3d relate11(
      Eigen::Vector3d(0.1, 0, 0), Eigen::Quaterniond::Identity());  // intra
  cartographer::transform::Rigid3d relate12(
      Eigen::Vector3d(2.26, 0, 0),
      Eigen::Quaterniond::Identity());  // inter to submap2
  cartographer::transform::Rigid3d relate13(
      Eigen::Vector3d(0.8, 0, 0), Eigen::Quaterniond::Identity());  // intra
  double translation_weight = 5e2;
  double rotation_weight = 1.6e3;
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 0),
      {relate1, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 1),
      {relate2, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 2),
      {relate3, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 3),
      {relate4, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTER_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 1),
      cartographer::mapping::NodeId(0, 3),
      {relate5, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 4),
      {relate6, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTER_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 1),
      cartographer::mapping::NodeId(0, 4),
      {relate7, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 0),
      cartographer::mapping::NodeId(0, 5),
      {relate8, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTER_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 1),
      cartographer::mapping::NodeId(0, 5),
      {relate9, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 1),
      cartographer::mapping::NodeId(0, 6),
      {relate10, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTER_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 2),
      cartographer::mapping::NodeId(0, 6),
      {relate11, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 1),
      cartographer::mapping::NodeId(0, 7),
      {relate12, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTER_SUBMAP});
  constraints.push_back(cartographer::pose_graph::optimization::Constraint{
      cartographer::mapping::SubmapId(0, 2),
      cartographer::mapping::NodeId(0, 7),
      {relate13, translation_weight, rotation_weight},
      cartographer::pose_graph::optimization::Constraint::INTRA_SUBMAP});
  for (auto constraint : constraints) {
    LOG(INFO) << "constraint relative pose before is: "
              << constraint.pose.zbar_ij.DebugString();
  }
  std::map<int, cartographer::pose_graph::optimization::TrajectoryState>
      trajectory_state;
  trajectory_state[0] =
      cartographer::pose_graph::optimization::TrajectoryState::ACTIVE;
  optimization.Solve(constraints, trajectory_state);
  for (auto submap_id_data : optimization.submap_data()) {
    LOG(INFO) << "submap data is: " << submap_id_data.id
              << " data is: " << submap_id_data.data.global_pose.DebugString();
  }
  for (auto node_id_data : optimization.node_data()) {
    LOG(INFO) << "node data id: " << node_id_data.id
              << " data is: " << node_id_data.data.global_pose_2d.DebugString();
  }
  for (auto constraint : constraints) {
    LOG(INFO) << "constraint relative pose after is: "
              << constraint.pose.zbar_ij.DebugString();
  }
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}