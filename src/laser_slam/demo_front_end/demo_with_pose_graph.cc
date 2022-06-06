/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/map_builder.h"

int main(int argc, char** argv) {
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