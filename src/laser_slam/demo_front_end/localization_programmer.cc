/**
 * @file localization_programmer.cc
 * @author Feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-09-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "laser_slam/localization_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::LocalizationNode> localization =
      std::make_shared<cartographer::laser_slam::LocalizationNode>();
  rclcpp::spin(localization->get_node_base_interface());
  rclcpp::shutdown();
}