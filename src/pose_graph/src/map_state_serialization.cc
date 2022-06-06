/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_graph/map_state_serialization.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {

void MapStateSerialization::SerializeSubmaps(
    const mapping::MapById<SubmapId, SubmapData>& submap_data,
    bool include_unfinished_submap,
    stream::ProtoStreamWriterInterface* const writer) {
  for (const auto& submap_id_data : submap_data) {
    if (!include_unfinished_submap &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    protos::mapping::proto::PoseGraph proto;
    protos::mapping::proto::SubmapData submap;
    protos::transform::proto::Rigid3d global_pose;
    global_pose = transform::ToProto(submap_id_data.data.pose);
    *submap.mutable_submap() = submap_id_data.data.submap->ToProto(true);
    submap.mutable_id()->set_submap_index(submap_id_data.id.submap_index);
    submap.mutable_id()->set_trajectory_id(submap_id_data.id.trajectory_id);
    *submap.mutable_global_pose() = global_pose;
    *proto.add_submaps() = submap;
    writer->WriteProto(proto);
  }
}

void MapStateSerialization::SerializeTrajectoryNode(
    const mapping::MapById<NodeId, TrajectoryNode>& node_data,
    stream::ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : node_data) {
    protos::mapping::proto::PoseGraph pose_graph;
    protos::mapping::proto::Trajectory proto;
    protos::mapping::proto::TrajectoryNode node;
    node.set_node_index(node_id_data.id.node_index);
    *node.mutable_global_pose() =
        transform::ToProto(node_id_data.data.global_pose);
    protos::mapping::proto::TrajectoryNodeData data;
    data.set_timestamp(
        common::ToUniversal(node_id_data.data.constant_data->time));
    *data.mutable_gravity_alignment() =
        transform::ToProto(node_id_data.data.constant_data->gravity_alignment);
    *data.mutable_local_pose() =
        transform::ToProto(node_id_data.data.constant_data->local_pose);
    *node.mutable_data() = data;
    proto.set_trajectory_id(node_id_data.id.trajectory_id);
    *proto.mutable_node() = node;
    *pose_graph.add_trajectory() = proto;
    writer->WriteProto(pose_graph);
  }
}

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer
