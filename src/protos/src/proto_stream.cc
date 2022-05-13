/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <glog/logging.h>

#include "protos/proto_stream.h"

namespace cartographer {
namespace stream {
namespace {

// First eight bytes to identify our proto stream format.
const uint64 kMagic = 0x7b1d1f7b5bf501db;

void WriteSizeAsLittleEndian(uint64 size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

bool ReadSizeAsLittleEndian(std::istream* in, uint64* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace
ProtoStreamWriter::ProtoStreamWriter(const std::string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

void ProtoStreamWriter::Write(const std::string& uncompressed_data) {
  std::string compressed_data = uncompressed_data;
  // common::FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

void ProtoStreamWriter::WriteProto(const google::protobuf::Message& proto) {
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  Write(uncompressed_data);
}

bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const std::string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64 magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  CHECK(in_.good()) << "Failed to open proto stream '" << filename << "'.";
}

bool ProtoStreamReader::Read(std::string* decompressed_data) {
  uint64 compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    LOG(INFO) << "1";
    return false;
  }
  std::string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    LOG(INFO) << "2";
    return false;
  }
  LOG(INFO) << "3";
  *decompressed_data = compressed_data;
  // common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

bool ProtoStreamReader::ReadProto(google::protobuf::Message* proto) {
  std::string decompressed_data;
  return Read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoStreamReader::eof() const { return in_.eof(); }
}  // namespace stream
}  // namespace cartographer