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
