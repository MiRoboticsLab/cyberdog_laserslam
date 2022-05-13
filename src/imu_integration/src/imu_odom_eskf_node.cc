/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "imu_integration/imu_odom_eskf_node.h"

namespace imu_integration {}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<imu_integration::ImuOdomEskfNode> eskf_ptr =
      std::make_shared<imu_integration::ImuOdomEskfNode>();
  eskf_ptr->Initialization();
  eskf_ptr->Start();
  rclcpp::spin(eskf_ptr);
  rclcpp::shutdown();
  eskf_ptr->Stop();
  return 0;
}