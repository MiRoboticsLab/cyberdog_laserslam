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
#include <memory>

#include "laser_slam/map_builder.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::MapBuilderNode> map_builder =
    std::make_shared<cartographer::laser_slam::MapBuilderNode>(
    0, "laser_slam", "/home/zfx/global_map", 4);
  map_builder->Initialization();
  executor.add_node(map_builder);
  executor.spin();
  rclcpp::shutdown();
  map_builder->SaveFinalMap();
  return 0;
}
