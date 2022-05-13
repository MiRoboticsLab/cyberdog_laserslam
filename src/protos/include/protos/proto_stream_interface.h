/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef PROTOS_PROTO_STREAM_INTERFACE_H_
#define PROTOS_PROTO_STREAM_INTERFACE_H_
#include <google/protobuf/message.h>

#include "common/port.h"

namespace cartographer {
namespace stream {
class ProtoStreamWriterInterface {
 public:
  virtual ~ProtoStreamWriterInterface() {}

  // Serializes, compressed and writes the 'proto' to the file.
  virtual void WriteProto(const google::protobuf::Message& proto) = 0;

  // This should be called to check whether writing was successful.
  virtual bool Close() = 0;
};
class ProtoStreamReaderInterface {
 public:
  ProtoStreamReaderInterface() = default;
  virtual ~ProtoStreamReaderInterface() {}

  // Deserialize compressed proto from the pb stream.
  virtual bool ReadProto(google::protobuf::Message* proto) = 0;

  // 'End-of-file' marker for the pb stream.
  virtual bool eof() const = 0;
};
}  // namespace stream
}  // namespace cartographer

#endif  // PROTOS_PROTO_STREAM_INTERFACE_H_
