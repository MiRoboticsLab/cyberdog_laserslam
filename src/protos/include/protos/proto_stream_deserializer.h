/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef PROTOS_PROTO_STREAM_DESERIALIZER_H_
#define PROTOS_PROTO_STREAM_DESERIALIZER_H_
#include <glog/logging.h>

#include "protos/proto_stream_interface.h"
#include "mapping/pose_graph.pb.h"

namespace cartographer {
namespace stream {
// Helper function for deserializing the PoseGraph from a proto stream file.
protos::mapping::proto::PoseGraph DeserializePoseGraphFromFile(
    const std::string& file_name);

// Helper for deserializing a previously serialized mapping state from a
// proto stream, abstracting away the format parsing logic.
class ProtoStreamDeserializer {
 public:
  explicit ProtoStreamDeserializer(ProtoStreamReaderInterface* const reader);

  ProtoStreamDeserializer(const ProtoStreamDeserializer&) = delete;
  ProtoStreamDeserializer& operator=(const ProtoStreamDeserializer&) = delete;
  ProtoStreamDeserializer(ProtoStreamDeserializer&&) = delete;

  protos::mapping::proto::PoseGraphHeader& header() { return header_; }

  protos::mapping::proto::PoseGraph& pose_graph() { return pose_graph_; }
  const protos::mapping::proto::PoseGraph& pose_graph() const {
    return pose_graph_;
  }

  // Reads the next `SerializedData` message of the ProtoStream into `data`.
  // Returns `true` if the message was successfully read or `false` in case
  // there are no-more messages or an error occurred.
  bool ReadNextSerializedData(protos::mapping::proto::PoseGraph* data);

 private:
  ProtoStreamReaderInterface* reader_;

  protos::mapping::proto::PoseGraphHeader header_;
  protos::mapping::proto::PoseGraph pose_graph_;
};
}  // namespace stream
}  // namespace cartographer

#endif  // PROTOS_PROTO_STREAM_DESERIALIZER_H_
