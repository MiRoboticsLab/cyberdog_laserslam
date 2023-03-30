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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/thread_pool.h"
#include "laser_slam/local_slam.hpp"
#include "laser_slam/final_map_generator.hpp"
#include "laser_slam/base_data/grid_for_navigation.hpp"
#include "laser_slam/base_data/pose_recorder.hpp"
#include "pose_graph/bundle_adjustment.h"
#include "protos/proto_stream_interface.h"
#include "protos/proto_stream.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "visualization/msg/matching_result.hpp"
#include "tf2/buffer_core.h"

namespace cartographer
{
namespace laser_slam
{
class RelocTest : public rclcpp::Node
{
public:
  RelocTest() {}
  virtual ~RelocTest() {}

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;
};

}  // namespace laser_slam
}  // namespace cartographer
