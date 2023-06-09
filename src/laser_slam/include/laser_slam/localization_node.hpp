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

#ifndef LASER_SLAM__LOCALIZATION_NODE_HPP_
#define LASER_SLAM__LOCALIZATION_NODE_HPP_
#include <condition_variable>
#include <mutex>
#include <memory>
#include <thread>
#include <deque>
#include <string>

#include "laser_slam/localization.hpp"
#include "laser_slam/base_data/pose_recorder.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cyberdog_visions_interfaces/srv/reloc.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2/buffer_core.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_util/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/time.h>
#include <tf2_ros/async_buffer_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>


//  using namespace std::chrono_literals;

namespace cartographer
{
namespace laser_slam
{
class LocalizationNode : public nav2_util::LifecycleNode
{
public:
  LocalizationNode();
  virtual ~LocalizationNode();

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu);

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laser);

  void PosePcCallBack(
    const transform::Rigid3d & pose,
    const sensor::RangeData & pc);
  
  void PublishPoseThread();

  void StartLocationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void StopLocationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool is_on_active_status_ = false;
  int reloc_id_;
  std::mutex job_mutex_;
  int64 last_laser_time_;
  std::string pose_save_path_;
  LocalizationParam localization_param_;

  // ros
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr location_mode_service_;
  rclcpp::Client<cyberdog_visions_interfaces::srv::Reloc>::SharedPtr
    reloc_client_;
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pc_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
    reloc_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;
  laser_geometry::LaserProjection projector_;
  rclcpp::TimerBase::SharedPtr tf_publisher_;

  // localization process
  LocalizationPtr localization_;
  Eigen::Matrix3d transform_;
  transform::Rigid3d laser_t_odom_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_location_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_location_service_;
  std::shared_ptr<std::thread> reloc_thread_;
  std::condition_variable job_condvar_;
  JobQueue jobs_;
  std::shared_ptr<PoseRecorder> pose_recorder_;
};
}   // namespace laser_slam
}   // namespace cartographer

#endif   // LASER_SLAM__LOCALIZATION_NODE_HPP_
