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
#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "server/reloc.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using protos::server::proto::Reloc;
using protos::server::proto::RelocReply;
using protos::server::proto::RelocRequest;

class RelocServiceImpl final : public Reloc::Service {
  Status GetRelocPose(ServerContext* context, const RelocRequest* request,
                      RelocReply* reply) override {
    protos::server::proto::Point pt_proto;
    // pt_proto.set_x(1.021);
    // pt_proto.set_y(0.8613);
    pt_proto.set_x(0.0);
    pt_proto.set_y(0.0);
    pt_proto.set_z(0.0);
    *reply->mutable_position() = pt_proto;
    protos::server::proto::Quaternion q_proto;
    q_proto.set_w(1.0);
    q_proto.set_x(0.0);
    q_proto.set_y(0.0);
    q_proto.set_z(0.0);
    // q_proto.set_w(-0.94325);
    // q_proto.set_x(0.0138);
    // q_proto.set_y(-0.000217);
    // q_proto.set_z(-0.332);
    *reply->mutable_bearing() = q_proto;
    int32_t time = 16000000;
    reply->set_timestamp(time);
    return Status::OK;
  }
};

void RunServer() {
  std::string server_adress("0.0.0.0:50051");
  RelocServiceImpl service;
  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  builder.AddListeningPort(server_adress, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  server->Wait();
}

int main(int argc, char** argv) {
  RunServer();

  return 0;
}
