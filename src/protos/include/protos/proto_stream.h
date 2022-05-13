/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef PROTOS_PROTO_STREAM_H_
#define PROTOS_PROTO_STREAM_H_
#include <fstream>

#include <google/protobuf/message.h>

#include "protos/proto_stream_interface.h"

namespace cartographer {
namespace stream {
// A simple writer of a compressed sequence of protocol buffer messages to a
// file. The format is not intended to be compatible with any other format used
// outside of Cartographer.
//
// TODO(whess): Compress the file instead of individual messages for better
// compression performance? Should we use LZ4?
class ProtoStreamWriter : public ProtoStreamWriterInterface {
 public:
  ProtoStreamWriter(const std::string& filename);
  ~ProtoStreamWriter() = default;

  ProtoStreamWriter(const ProtoStreamWriter&) = delete;
  ProtoStreamWriter& operator=(const ProtoStreamWriter&) = delete;

  void WriteProto(const google::protobuf::Message& proto) override;
  bool Close() override;

 private:
  void Write(const std::string& uncompressed_data);

  std::ofstream out_;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReader : public ProtoStreamReaderInterface {
 public:
  explicit ProtoStreamReader(const std::string& filename);
  ~ProtoStreamReader() = default;

  ProtoStreamReader(const ProtoStreamReader&) = delete;
  ProtoStreamReader& operator=(const ProtoStreamReader&) = delete;

  bool ReadProto(google::protobuf::Message* proto) override;
  bool eof() const override;

 private:
  bool Read(std::string* decompressed_data);

  std::ifstream in_;
};

}  // namespace protos
}  // namespace cartographer

#endif  // PROTOS_PROTO_STREAM_H_
