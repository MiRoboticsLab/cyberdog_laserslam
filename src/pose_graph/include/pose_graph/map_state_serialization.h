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
