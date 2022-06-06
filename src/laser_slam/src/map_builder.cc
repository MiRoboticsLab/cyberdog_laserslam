/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/map_builder.h"

namespace cartographer {
namespace laser_slam {
bool MapBuilderNode::Initialization() {
  // Get parameters from yaml
  LocalSlamParam param;
  this->declare_parameter("use_imu");
  this->get_parameter("use_imu", param.use_imu);
  this->declare_parameter("use_real_time_correlative_scan_matching");
  this->get_parameter("use_real_time_correlative_scan_matching",
                      param.use_real_time_correlative_scan_matching);
  this->declare_parameter("num_accumulated");
  this->get_parameter("num_accumulated", param.num_accumulated);
  this->declare_parameter("use_filter");
  this->get_parameter("use_filter", param.use_filter);
  this->declare_parameter("gravity_constant");
  this->get_parameter("gravity_constant", param.gravity_constant);
  this->declare_parameter("pose_duration_time");
  this->get_parameter("pose_duration_time", param.pose_duration_time);
  this->declare_parameter("max_time_seconds");
  this->get_parameter("max_time_seconds", param.max_time_seconds);
  this->declare_parameter("max_distance_meters");
  this->get_parameter("max_distance_meters", param.max_distance_meters);
  this->declare_parameter("max_angle_radians");
  this->get_parameter("max_angle_radians", param.max_angle_radians);
  this->declare_parameter("min_range");
  this->get_parameter("min_range", param.min_range);
  this->declare_parameter("max_range");
  this->get_parameter("max_range", param.max_range);
  this->declare_parameter("missing_data_ray_length");
  this->get_parameter("missing_data_ray_length", param.missing_data_ray_length);
  this->declare_parameter("voxel_filter_size");
  this->get_parameter("voxel_filter_size", param.voxel_filter_size);
  this->declare_parameter("min_num_points");
  this->get_parameter("min_num_points", param.min_num_points);
  this->declare_parameter("max_length");
  this->get_parameter("max_length", param.max_length);
  this->declare_parameter("max_range_scale");
  this->get_parameter("max_range_scale", param.max_range_scale);
  this->declare_parameter("submap_param.grid_insert_type");
  this->get_parameter("submap_param.grid_insert_type",
                      param.submap_param.grid_insert_type);
  this->declare_parameter("submap_param.num_range_data");
  this->get_parameter("submap_param.num_range_data",
                      param.submap_param.num_range_data);
  this->declare_parameter("submap_param.grid_type");
  this->get_parameter("submap_param.grid_type", param.submap_param.grid_type);
  this->declare_parameter("submap_param.resolution");
  this->get_parameter("submap_param.resolution", param.submap_param.resolution);
  this->declare_parameter(
      "submap_param.probability_inserter_param.hit_probability");
  this->get_parameter(
      "submap_param.probability_inserter_param.hit_probability",
      param.submap_param.probability_insert_param.hit_probability);
  this->declare_parameter(
      "submap_param.probability_inserter_param.miss_probability");
  this->get_parameter(
      "submap_param.probability_inserter_param.miss_probability",
      param.submap_param.probability_insert_param.miss_probability);
  this->declare_parameter(
      "submap_param.probability_inserter_param.insert_free");
  this->get_parameter("submap_param.probability_inserter_param.insert_free",
                      param.submap_param.probability_insert_param.insert_free);
  this->declare_parameter("ceres_scan_matching_param.use_nonmonotonic_steps");
  this->get_parameter("ceres_scan_matching_param.use_nonmonotonic_steps",
                      param.ceres_param.use_nonmonotonic_steps);
  this->declare_parameter("ceres_scan_matching_param.max_num_iterations");
  this->get_parameter("ceres_scan_matching_param.max_num_iterations",
                      param.ceres_param.max_num_iterations);
  this->declare_parameter("ceres_scan_matching_param.num_threads");
  this->get_parameter("ceres_scan_matching_param.num_threads",
                      param.ceres_param.num_threads);
  this->declare_parameter("ceres_scan_matching_param.occupied_space_weight");
  this->get_parameter("ceres_scan_matching_param.occupied_space_weight",
                      param.ceres_param.occupied_space_weight);
  this->declare_parameter("ceres_scan_matching_param.translation_weight");
  this->get_parameter("ceres_scan_matching_param.translation_weight",
                      param.ceres_param.translation_weight);
  this->declare_parameter("ceres_scan_matching_param.rotation_weight");
  this->get_parameter("ceres_scan_matching_param.rotation_weight",
                      param.ceres_param.rotation_weight);
  this->declare_parameter(
      "real_time_correlative_scan_matching_param.linear_search_window");
  this->get_parameter(
      "real_time_correlative_scan_matching_param.linear_search_window",
      param.real_time_param.linear_search_window);
  this->declare_parameter(
      "real_time_correlative_scan_matching_param.angular_search_window");
  this->get_parameter(
      "real_time_correlative_scan_matching_param.angular_search_window",
      param.real_time_param.angular_search_window);
  this->declare_parameter(
      "real_time_correlative_scan_matching_param.translation_delta_cost_"
      "weight");
  this->get_parameter(
      "real_time_correlative_scan_matching_param.translation_delta_cost_"
      "weight",
      param.real_time_param.translation_delta_cost_weight);
  this->declare_parameter(
      "real_time_correlative_scan_matching_param.rotation_delta_cost_weight");
  this->get_parameter(
      "real_time_correlative_scan_matching_param.rotation_delta_cost_weight",
      param.real_time_param.rotation_delta_cost_weight);
  local_slam_.reset(new LocalSlam(param));

  PoseGraph2DParam pose_graph_param;
  this->declare_parameter("pose_graph_param.optimize_every_n_nodes");
  this->get_parameter("pose_graph_param.optimize_every_n_nodes",
                      pose_graph_param.optimize_every_n_nodes);
  this->declare_parameter("pose_graph_param.max_num_final_iterations");
  this->get_parameter("pose_graph_param.max_num_final_iterations",
                      pose_graph_param.max_num_final_iterations);
  this->declare_parameter("pose_graph_param.matcher_rotation_weight");
  this->get_parameter("pose_graph_param.matcher_rotation_weight",
                      pose_graph_param.matcher_rotation_weight);
  LOG(INFO) << "rotation weight: " << pose_graph_param.matcher_rotation_weight;
  this->declare_parameter("pose_graph_param.matcher_translation_weight");
  this->get_parameter("pose_graph_param.matcher_translation_weight",
                      pose_graph_param.matcher_translation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.min_score");
  this->get_parameter("pose_graph_param.constraint_builder_param.min_score",
                      pose_graph_param.constraint_builder_param.min_score);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.log_matches");
  this->get_parameter("pose_graph_param.constraint_builder_param.log_matches",
                      pose_graph_param.constraint_builder_param.log_matches);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_rotation_weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_rotation_weight",
      pose_graph_param.constraint_builder_param.loop_closure_rotation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_translation_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_translation_"
      "weight",
      pose_graph_param.constraint_builder_param
          .loop_closure_translation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.max_constraint_distance");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.max_constraint_distance",
      pose_graph_param.constraint_builder_param.max_constraint_distance);
  this->declare_parameter("pose_graph_param.constraint_builder_param.ratio");
  this->get_parameter("pose_graph_param.constraint_builder_param.ratio",
                      pose_graph_param.constraint_builder_param.ratio);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.max_num_"
      "iterations");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.max_num_"
      "iterations",
      pose_graph_param.constraint_builder_param.ceres_param.max_num_iterations);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.num_threads");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.num_threads",
      pose_graph_param.constraint_builder_param.ceres_param.num_threads);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.use_nonmonotonic_"
      "steps");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.use_nonmonotonic_"
      "steps",
      pose_graph_param.constraint_builder_param.ceres_param
          .use_nonmonotonic_steps);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.occupied_space_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.occupied_space_"
      "weight",
      pose_graph_param.constraint_builder_param.ceres_param
          .occupied_space_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.translation_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.translation_"
      "weight",
      pose_graph_param.constraint_builder_param.ceres_param.translation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.rotation_weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_param.rotation_weight",
      pose_graph_param.constraint_builder_param.ceres_param.rotation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
      "depth");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
      "depth",
      pose_graph_param.constraint_builder_param.fast_param
          .branch_and_bound_depth);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
      "window");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
      "window",
      pose_graph_param.constraint_builder_param.fast_param
          .angular_search_window);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
      "window");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
      "window",
      pose_graph_param.constraint_builder_param.fast_param
          .linear_search_window);
  this->declare_parameter("pose_graph_param.optimization_param.huber_scale");
  this->get_parameter("pose_graph_param.optimization_param.huber_scale",
                      pose_graph_param.optimization_param.huber_scale);
  this->declare_parameter(
      "pose_graph_param.optimization_param.max_num_iterations");
  this->get_parameter("pose_graph_param.optimization_param.max_num_iterations",
                      pose_graph_param.optimization_param.max_num_iterations);
  this->declare_parameter("pose_graph_param.optimization_param.num_threads");
  this->get_parameter("pose_graph_param.optimization_param.num_threads",
                      pose_graph_param.optimization_param.num_threads);
  this->declare_parameter(
      "pose_graph_param.optimization_param.report_full_summary");
  this->get_parameter("pose_graph_param.optimization_param.report_full_summary",
                      pose_graph_param.optimization_param.report_full_summary);
  pose_graph_.reset(new pose_graph::optimization::BundleAdjustment(
      pose_graph_param, &thread_pool_));

  // subscribe and publisher initialization
  callback_imu_subscriber_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_laser_subscriber_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_odometry_subscriber_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_imu_opt = rclcpp::SubscriptionOptions();
  sub_imu_opt.callback_group = callback_imu_subscriber_;
  auto sub_odom_opt = rclcpp::SubscriptionOptions();
  sub_odom_opt.callback_group = callback_odometry_subscriber_;
  auto sub_laser_opt = rclcpp::SubscriptionOptions();
  sub_laser_opt.callback_group = callback_laser_subscriber_;
  std::string imu_topic("");
  this->declare_parameter("imu_topic", imu_topic);
  this->get_parameter("imu_topic", imu_topic);
  LOG(INFO) << imu_topic;
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&MapBuilderNode::AddImuData, this, std::placeholders::_1),
      sub_imu_opt);
  std::string odometry_topic("");
  this->declare_parameter("odometry_topic", odometry_topic);
  this->get_parameter("odometry_topic", odometry_topic);
  LOG(INFO) << odometry_topic;
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&MapBuilderNode::AddOdometryData, this, std::placeholders::_1),
      sub_odom_opt);
  std::string laser_topic("");
  this->declare_parameter("laser_scan_topic", laser_topic);
  this->get_parameter("laser_scan_topic", laser_topic);
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, rclcpp::SensorDataQoS(),
      std::bind(&MapBuilderNode::AddRangeData, this, std::placeholders::_1),
      sub_laser_opt);
  std::vector<double> tmp{0, -1, 0, 0, 0, -1, 1, 0, 0};
  this->declare_parameter("imu_to_odom");
  rclcpp::Parameter tf = this->get_parameter("imu_to_odom");
  tmp = tf.as_double_array();
  imu_t_w_ << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7],
      tmp[8];
  LOG(INFO) << "transform is: " << imu_t_w_;
  grid_.reset(
      new GridForNavigation(param.submap_param.resolution,
                            param.submap_param.probability_insert_param));
  publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("laser_pose", 10);
  return true;
}

void MapBuilderNode::AddImuData(const sensor_msgs::msg::Imu::SharedPtr imu) {
  sensor::ImuData imu_meas;
  Eigen::Vector3d acc, gyro;
  acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
      imu->linear_acceleration.z;
  gyro << imu->angular_velocity.x, imu->angular_velocity.y,
      imu->angular_velocity.z;
  imu_meas.linear_acceleration = (acc.transpose() * imu_t_w_).transpose();
  imu_meas.angular_velocity = (gyro.transpose() * imu_t_w_).transpose();
  imu_meas.time = common::FromUniversal(
      (imu->header.stamp.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (imu->header.stamp.nanosec + 50) / 100);
  local_slam_->AddImuData(imu_meas);
}

void MapBuilderNode::AddOdometryData(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  Eigen::Vector3d position;
  Eigen::Quaterniond angular;
  position << odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0;
  angular = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  sensor::OdometryData odom_meas;
  transform::Rigid3d pose(position, angular);
  odom_meas.pose = pose;
  odom_meas.time = common::FromUniversal(
      (odom->header.stamp.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (odom->header.stamp.nanosec + 50) / 100);
  local_slam_->AddOdometryData(odom_meas);
}

void MapBuilderNode::AddRangeData(
    const sensor_msgs::msg::LaserScan::SharedPtr laser) {
  std::vector<sensor::RangefinderPoint> points;
  sensor_msgs::msg::PointCloud2 cloud;
  common::Time time = common::FromRosTime(laser->header.stamp);
  projector_.projectLaser(*laser, cloud, 20.0);
  pcl::PointCloud<pcl::PointXYZ> raw_cloud;
  pcl::fromROSMsg(cloud, raw_cloud);
  for (size_t i = 0; i < raw_cloud.points.size(); ++i) {
    sensor::RangefinderPoint pt;
    pt.position.x() = raw_cloud.points[i].x;
    pt.position.y() = raw_cloud.points[i].y;
    pt.position.z() = raw_cloud.points[i].z;
    points.push_back(pt);
  }
  if (local_slam_->LatestPoseExtrapolatorTime() == common::Time::min() ||
      time < local_slam_->LatestPoseExtrapolatorTime()) {
    LOG(INFO) << "return cause by time"
              << local_slam_->LatestPoseExtrapolatorTime();
    return;
  }
  sensor::PointCloud pc(time, points);
  std::unique_ptr<MatchingResult> local_match_result =
      local_slam_->AddRangeData(pc);
  if (local_match_result != nullptr) {
    if (local_match_result->insertion_result != nullptr) {
      std::shared_ptr<const pose_graph::optimization::TrajectoryNode::Data>
          node_data = local_match_result->insertion_result->node_constant_data;
      auto& submap_datas =
          local_match_result->insertion_result->insertion_submaps;
      //   mapping::NodeId node_id =
      //       pose_graph_->AddNode(node_data, trajectory_id_, submap_datas);
      mapping::NodeId node_id(trajectory_id_, index);
      ++index;
      id_local_range_data_[node_id] = local_match_result->range_data_in_local;
      geometry_msgs::msg::PoseStamped pose_pub;
      pose_pub.header.frame_id = "laser";
      pose_pub.header.stamp = laser->header.stamp;
      pose_pub.pose.orientation.x =
          local_match_result->local_pose.rotation().x();
      pose_pub.pose.orientation.y =
          local_match_result->local_pose.rotation().y();
      pose_pub.pose.orientation.z =
          local_match_result->local_pose.rotation().z();
      pose_pub.pose.orientation.w =
          local_match_result->local_pose.rotation().w();
      pose_pub.pose.position.x =
          local_match_result->local_pose.translation().x();
      pose_pub.pose.position.y =
          local_match_result->local_pose.translation().y();
      pose_pub.pose.position.z =
          local_match_result->local_pose.translation().z();
      publisher_->publish(pose_pub);
      if (pose_callback_) {
        /**
         * Get the latest pose from pose graph
         */
        pose_callback_(
            pose_graph_->GetLatestNodeData(trajectory_id_).global_pose);
      }
    }
  }
}

void MapBuilderNode::SetRealTimePoseCallBack(RealTimePoseCallback callback) {
  pose_callback_ = callback;
}

void MapBuilderNode::SaveFinalMap() {
  LOG(INFO) << "Run final optimization";
  pose_graph_->RunFinalOptimization();
  LOG(INFO) << "finish optimization";
  const auto pose_graph_data = pose_graph_->pose_graph_data();
  std::vector<sensor::RangeData> range_datas;
  // transform all node range data in local to global
  auto begin_it =
      pose_graph_data.trajectory_nodes.BeginOfTrajectory(trajectory_id_);
  auto end_it =
      pose_graph_data.trajectory_nodes.EndOfTrajectory(trajectory_id_);
  for (; begin_it != end_it; ++begin_it) {
    transform::Rigid3d local_to_global =
        pose_graph_data.trajectory_nodes.at(begin_it->id).global_pose *
        pose_graph_data.trajectory_nodes.at(begin_it->id)
            .constant_data->local_pose.inverse();
    auto range_data = id_local_range_data_.at(begin_it->id);
    LOG(INFO) << "delta pose is: " << local_to_global.DebugString();
    auto pc =
        sensor::TransformRangeData(range_data, local_to_global.cast<float>());
    range_datas.push_back(range_data);
  }
  grid_->RayCastByProbability(range_datas);
  grid_->WritePgmByProbabilityGrid(file_name_);
  LOG(INFO) << "Write pgm";
}

}  // namespace laser_slam
}  // namespace cartographer