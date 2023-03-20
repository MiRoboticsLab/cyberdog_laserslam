// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <memory>

#include "laser_slam/map_loader.hpp"

namespace cartographer
{
namespace laser_slam
{
namespace
{
pose_graph::optimization::Constraint::Tag FromProto(
  const protos::mapping::proto::PoseGraph::Constraint::Tag & tag_proto)
{
  switch (tag_proto) {
    case protos::mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP:
      return pose_graph::optimization::Constraint::INTRA_SUBMAP;
    case protos::mapping::proto::PoseGraph::Constraint::INTER_SUBMAP:
      return pose_graph::optimization::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag from proto";
}

mapping::NodeId FromProto(const protos::mapping::proto::NodeId & node_id_proto)
{
  mapping::NodeId node_id(node_id_proto.trajectory_id(),
    node_id_proto.node_index());
  return node_id;
}

mapping::SubmapId FromProto(
  const protos::mapping::proto::SubmapId & submap_id_proto)
{
  mapping::SubmapId submap_id(submap_id_proto.trajectory_id(),
    submap_id_proto.submap_index());
  return submap_id;
}

pose_graph::optimization::Constraint FromProto(
  const protos::mapping::proto::PoseGraph::Constraint & constraint_proto)
{
  pose_graph::optimization::Pose pose{
    transform::ToRigid3(constraint_proto.relative_pose()),
    constraint_proto.translation_weight(),
    constraint_proto.rotation_weight()};
  pose_graph::optimization::Constraint constraint{
    FromProto(constraint_proto.submap_id()),
    FromProto(constraint_proto.node_id()), pose,
    FromProto(constraint_proto.tag())};
  return constraint;
}

pose_graph::optimization::TrajectoryNode::Data FromProto(
  const protos::mapping::proto::TrajectoryNodeData & data_proto)
{
  return pose_graph::optimization::TrajectoryNode::Data{
    common::FromUniversal(data_proto.timestamp()),
    transform::ToEigen(data_proto.gravity_alignment()),
    sensor::PointCloud(data_proto.filtered_gravity_aligned_point_cloud()),
    transform::ToRigid3(data_proto.local_pose())};
}

}  // namespace

MapLoader::MapLoader(const stream::ProtoStreamDeserializer & deserializer)
{
  auto & pose_graph = deserializer.pose_graph();
  auto trajectory_id = pose_graph.trajectory_id();
  trajectory_id_ = trajectory_id;
  for (const auto constraint : pose_graph.constraint()) {
    pose_graph::optimization::Constraint constraint_s = FromProto(constraint);
    constraints_.push_back(constraint_s);
  }
  for (const auto submap : pose_graph.submaps()) {
    pose_graph::optimization::SubmapId id = FromProto(submap.id());
    pose_graph::optimization::SubmapData data;
    data.pose = transform::ToRigid3(submap.global_pose());
    std::shared_ptr<mapping::Submap2D> map =
      std::make_shared<mapping::Submap2D>(
      submap.submap(),
      &conversion_tables_);
    data.submap = map;
    id_submap_data_.Insert(id, data);
  }
  for (const auto trajectory : pose_graph.trajectory()) {
    mapping::NodeId id(pose_graph.trajectory_id(), trajectory.node_index());
    pose_graph::optimization::TrajectoryNode data;
    data.constant_data =
      std::make_shared<const pose_graph::optimization::TrajectoryNode::Data>(
      FromProto(trajectory.data()));
    data.global_pose = transform::ToRigid3(trajectory.global_pose());
    id_node_data_.Insert(id, data);
  }
}

MapLoader::~MapLoader() {}
}  // namespace laser_slam
}  // namespace cartographer
