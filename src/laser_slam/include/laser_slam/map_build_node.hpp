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

#ifndef LASER_SLAM__MAP_BUILD_NODE_HPP_
#define LASER_SLAM__MAP_BUILD_NODE_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <string>
#include <map>

#include "common/thread_pool.h"
#include "laser_slam/base_data/grid_for_display.hpp"
#include "laser_slam/base_data/grid_for_navigation.hpp"
#include "laser_slam/base_data/pose_recorder.hpp"
#include "laser_slam/base_data/submap_points_batch.hpp"
#include "laser_slam/final_map_generator.hpp"
#include "laser_slam/local_slam.hpp"
#include "laser_slam/map_server_node.hpp"
#include "pose_graph/bundle_adjustment.h"
#include "protos/proto_stream.h"
#include "protos/proto_stream_interface.h"
#include "rviz_display/probability_grid_conversion.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization/srv/stop.hpp"
namespace cartographer
{
namespace laser_slam
{
class MapBuildNode : public nav2_util::LifecycleNode
{
public:
  explicit MapBuildNode(int thread_num);
  virtual ~MapBuildNode();

protected:
  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laser);

  void StartMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Stop mapping service with bool value to determine if save map or not
  void StopMappingCallback(
    const std::shared_ptr<visualization::srv::Stop::Request> request,
    std::shared_ptr<visualization::srv::Stop::Response> response);

  bool SaveMap(bool save_map, const std::string & map_name);

  void DisplayMapPublishPeriod();

  void SubmapCallback(
    const mapping::SubmapId & id,
    const std::shared_ptr<const mapping::Submap> & data);

  bool CheckDirectory(const std::string & path);

  void GetMapPathCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  bool is_on_active_status_;
  bool is_multi_map_mode_;
  bool is_map_name_got_;
  bool save_map_;
  double map_publish_period_sec_;
  int inserted_node_num_ = 0;
  bool in_laser_process_ = false;
  int64 last_laser_time_;
  LocalSlamParam local_slam_param_;
  PoseGraph2DParam pose_graph_param_;
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
    nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;

  std::string frame_id_;
  std::string map_save_path_;
  common::Time last_time_;
  std::map<mapping::NodeId, sensor::RangeData> id_data_;
  LocalSlamPtr local_slam_;
  common::ThreadPool thread_pool_;
  pose_graph::optimization::BundleAdjustmentPtr pose_graph_;
  std::shared_ptr<PoseRecorder> pose_recorder_;
  GridForNavigationPtr grid_;
  Eigen::Matrix3d transform_;
  transform::Rigid3d laser_t_odom_;
  laser_geometry::LaserProjection projector_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_mapping_service_;
  //   rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
  //   stop_mapping_service_;
  rclcpp::Service<visualization::srv::Stop>::SharedPtr stop_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr map_name_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_map_notify_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_map_notify_client_;
  rclcpp::TimerBase::SharedPtr grid_publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  GridForDisplayPtr display_ptr_;
  std::shared_ptr<MapServerNode> mapping_ptr_;
  mapping::SubmapId id_;
};
}  // namespace laser_slam
}  // namespace cartographer
#endif  // LASER_SLAM__MAP_BUILD_NODE_HPP_
