/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "protos/proto_stream_deserializer.h"
constexpr int kMappingStateSerializationFormatVersion = 2;
constexpr int kFormatVersionWithoutSubmapHistograms = 1;
namespace cartographer {
namespace stream {
namespace {
protos::mapping::proto::PoseGraphHeader ReadHeaderOrDie(
    ProtoStreamReaderInterface* const reader) {
  protos::mapping::proto::PoseGraphHeader header;
  CHECK(reader->ReadProto(&header)) << "Failed to read SerializationHeader.";
  return header;
}

bool IsVersionSupported(const protos::mapping::proto::PoseGraphHeader& header) {
  return header.format_version() == kMappingStateSerializationFormatVersion ||
         header.format_version() == kFormatVersionWithoutSubmapHistograms;
}
}  // namespace

ProtoStreamDeserializer::ProtoStreamDeserializer(
    ProtoStreamReaderInterface* const reader)
    : reader_(reader), header_(ReadHeaderOrDie(reader)) {
  CHECK(IsVersionSupported(header_)) << "Unsupported serialization format \""
                                     << header_.format_version() << "\"";
  LOG(INFO) << "serialization format is: " << header_.format_version();
  CHECK(ReadNextSerializedData(&pose_graph_))
      << "Serialized stream misses PoseGraph.";
}

bool ProtoStreamDeserializer::ReadNextSerializedData(
    protos::mapping::proto::PoseGraph* data) {
  return reader_->ReadProto(data);
}

}  // namespace stream
}  // namespace cartographer
