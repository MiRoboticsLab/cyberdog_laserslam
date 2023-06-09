// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef LASER_SLAM__MAP_BUILDER_HPP_
#define LASER_SLAM__MAP_BUILDER_HPP_
#include <functional>
#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

#include "laser_slam/local_slam.hpp"
#include "laser_slam/base_data/grid_for_navigation.hpp"
#include "pose_graph/bundle_adjustment.h"

// Build Map with laser imu odometry data
namespace cartographer
{
namespace laser_slam
{
class MapBuilderNode : public rclcpp::Node
{
public:
  using RealTimePoseCallback = std::function<void (const transform::Rigid3d &)>;
  MapBuilderNode(
    int trajectory_id, const std::string & node_name,
    const std::string & file_name, int thread_num)
  : Node(node_name),
    trajectory_id_(trajectory_id),
    file_name_(file_name),
    thread_pool_(thread_num),
    grid_(nullptr) {}
  virtual ~MapBuilderNode() {}

  // initial local slam and pose graph, get the parameter from server
  bool Initialization();

  // Run final optimization and save the final map
  void SaveFinalMap();

  void SetRealTimePoseCallBack(RealTimePoseCallback callback);

private:
  void AddImuData(const sensor_msgs::msg::Imu::SharedPtr imu);

  void AddOdometryData(const nav_msgs::msg::Odometry::SharedPtr odom);

  void AddRangeData(const sensor_msgs::msg::LaserScan::SharedPtr laser);

  int trajectory_id_;
  std::string file_name_;  // Path to save pgm map and proto map
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;
  pose_graph::optimization::BundleAdjustmentPtr pose_graph_;
  LocalSlamPtr local_slam_;
  Eigen::Matrix3d imu_t_w_;
  RealTimePoseCallback pose_callback_;
  common::ThreadPool thread_pool_;
  laser_geometry::LaserProjection projector_;
  GridForNavigationPtr grid_;
  std::map<mapping::NodeId, sensor::RangeData> id_local_range_data_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  int index = 0;
};
}   // namespace laser_slam
}   // namespace cartographer

#endif  // LASER_SLAM__MAP_BUILDER_HPP_
