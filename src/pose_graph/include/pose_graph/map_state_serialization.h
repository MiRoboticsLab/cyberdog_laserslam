/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_MAP_STATE_SERIALIZATION_H_
#define POSE_GRAPH_MAP_STATE_SERIALIZATION_H_
#include "protos/proto_stream_interface.h"
#include "protos/proto_stream.h"
#include "mapping/pose_graph.pb.h"
#include "pose_graph/data_set/pose_graph_data.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
class MapStateSerialization {
 public:
  MapStateSerialization() {}
  virtual ~MapStateSerialization() {}

 private:
  void SerializeTrajectoryNode(
      const mapping::MapById<NodeId, TrajectoryNode>& node_data,
      stream::ProtoStreamWriterInterface* const writer);
  void SerializeSubmaps(
      const mapping::MapById<SubmapId, SubmapData>& submap_data,
      bool include_unfinished_submap,
      stream::ProtoStreamWriterInterface* const writer);
  void SerializeConstraints(const std::vector<Constraint>& constraints,
                            stream::ProtoStreamWriterInterface* const writer);
};
}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_MAP_STATE_SERIALIZATION_H_
