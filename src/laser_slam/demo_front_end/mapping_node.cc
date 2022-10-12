/**
 * @file mapping_node.cc
 * @author Feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-07-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "laser_slam/map_build_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::MapBuildNode> local_slam =
      std::make_shared<cartographer::laser_slam::MapBuildNode>(4);
  executor.add_node(local_slam->get_node_base_interface());
  executor.spin();
  // rclcpp::spin(local_slam->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
