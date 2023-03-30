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
#ifndef PROTOS_CLIENT_H_
#define PROTOS_CLIENT_H_
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <grpcpp/grpcpp.h>

#include "common/time.h"
#include "server/reloc.pb.h"
#include "server/reloc.grpc.pb.h"

namespace cartographer {
namespace stream {
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
class Client {
 public:
  Client(std::shared_ptr<Channel> channel)
      : stub_(protos::server::proto::Reloc::NewStub(channel)) {}
  virtual ~Client() {}

  Status GetRelocPose(const std::string& user, common::Time* timestamp,
                      Eigen::Quaterniond* bearing, Eigen::Vector3d* position) {
    protos::server::proto::RelocRequest request;
    request.set_name(user);

    protos::server::proto::RelocReply reply;

    grpc::ClientContext context;

    Status status = stub_->GetRelocPose(&context, request, &reply);
    Eigen::Quaterniond bearing_result;
    Eigen::Vector3d position_result;
    if (status.ok()) {
      position_result = Eigen::Vector3d(
          reply.position().x(), reply.position().y(), reply.position().z());
      bearing_result.w() = reply.bearing().w();
      bearing_result.x() = reply.bearing().x();
      bearing_result.y() = reply.bearing().y();
      bearing_result.z() = reply.bearing().z();
      *bearing = bearing_result;
      *position = position_result;
      common::Time time = common::FromUniversal(reply.timestamp());
      *timestamp = time;
      std::cout << "ok" << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
    return status;
  }

 private:
  std::unique_ptr<protos::server::proto::Reloc::Stub> stub_;
};
typedef std::shared_ptr<Client> ClientPtr;
typedef std::shared_ptr<const Client> ClientConstPtr;
}  // namespace stream
}  // namespace cartographer

#endif  // PROTOS_CLIENT_H_
