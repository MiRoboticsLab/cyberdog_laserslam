/**
 * @file localization_node.cc
 * @author feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-08-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "laser_slam/localization_node.h"

namespace cartographer {
namespace laser_slam {
LocalizationNode::LocalizationNode()
    : nav2_util::LifecycleNode("localization", ""),
      reloc_id_(0),
      last_laser_time_(0),
      reloc_client_(nullptr),
      localization_(nullptr),
      reloc_thread_(nullptr),
      pose_recorder_(nullptr) {}

LocalizationNode::~LocalizationNode() {}

nav2_util::CallbackReturn LocalizationNode::on_configure(
    const rclcpp_lifecycle::State& state) {
  LocalSlamParam param;
  std::string frame_id("");
  this->declare_parameter("frame_id", frame_id);
  this->get_parameter("frame_id", frame_id);
  LOG(INFO) << "frame id: " << frame_id;
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

  ConstraintBuilderParam constraint_param;
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.min_score");
  this->get_parameter("pose_graph_param.constraint_builder_param.min_score",
                      constraint_param.min_score);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.reloc_min_score");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.reloc_min_score",
      constraint_param.reloc_min_score);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.log_matches");
  this->get_parameter("pose_graph_param.constraint_builder_param.log_matches",
                      constraint_param.log_matches);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_rotation_weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_rotation_weight",
      constraint_param.loop_closure_rotation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_translation_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.loop_closure_translation_"
      "weight",
      constraint_param.loop_closure_translation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.max_constraint_distance");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.max_constraint_distance",
      constraint_param.max_constraint_distance);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.max_reloc_constraint_"
      "distance");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.max_reloc_constraint_distance",
      constraint_param.max_reloc_constraint_distance);
  this->declare_parameter("pose_graph_param.constraint_builder_param.ratio");
  this->get_parameter("pose_graph_param.constraint_builder_param.ratio",
                      constraint_param.ratio);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.max_num_"
      "iterations");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.max_num_"
      "iterations",
      constraint_param.ceres_param.max_num_iterations);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.num_threads");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.num_threads",
      constraint_param.ceres_param.num_threads);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.use_nonmonotonic_"
      "steps");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.use_nonmonotonic_"
      "steps",
      constraint_param.ceres_param.use_nonmonotonic_steps);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.occupied_space_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.occupied_space_"
      "weight",
      constraint_param.ceres_param.occupied_space_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.translation_"
      "weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.translation_"
      "weight",
      constraint_param.ceres_param.translation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.rotation_weight");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.ceres_params.rotation_weight",
      constraint_param.ceres_param.rotation_weight);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
      "depth");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
      "depth",
      constraint_param.fast_param.branch_and_bound_depth);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
      "window");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
      "window",
      constraint_param.fast_param.angular_search_window);
  this->declare_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
      "window");
  this->get_parameter(
      "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
      "window",
      constraint_param.fast_param.linear_search_window);

  PoseGraph2DParam pose_graph_param;
  pose_graph_param.constraint_builder_param = constraint_param;
  OptimizationParam optimization_param;
  this->declare_parameter("pose_graph_param.optimization_param.huber_scale");
  this->get_parameter("pose_graph_param.optimization_param.huber_scale",
                      optimization_param.huber_scale);
  this->declare_parameter(
      "pose_graph_param.optimization_param.max_num_iterations");
  this->get_parameter("pose_graph_param.optimization_param.max_num_iterations",
                      optimization_param.max_num_iterations);
  this->declare_parameter("pose_graph_param.optimization_param.num_threads");
  this->get_parameter("pose_graph_param.optimization_param.num_threads",
                      optimization_param.num_threads);
  this->declare_parameter(
      "pose_graph_param.optimization_param.report_full_summary");
  this->get_parameter("pose_graph_param.optimization_param.report_full_summary",
                      optimization_param.report_full_summary);
  pose_graph_param.optimization_param = optimization_param;
  this->declare_parameter("pose_graph_param.optimize_every_n_nodes");
  this->get_parameter("pose_graph_param.optimize_every_n_nodes",
                      pose_graph_param.optimize_every_n_nodes);
  this->declare_parameter("pose_graph_param.max_num_final_iterations");
  this->get_parameter("pose_graph_param.max_num_final_iterations",
                      pose_graph_param.max_num_final_iterations);
  this->declare_parameter("pose_graph_param.matcher_rotation_weight");
  this->get_parameter("pose_graph_param.matcher_rotation_weight",
                      pose_graph_param.matcher_rotation_weight);
  this->declare_parameter("pose_graph_param.matcher_translation_weight");
  this->get_parameter("pose_graph_param.matcher_translation_weight",
                      pose_graph_param.matcher_translation_weight);
  this->declare_parameter("pose_graph_param.max_submaps_maintain");
  this->get_parameter("pose_graph_param.max_submaps_maintain",
                      pose_graph_param.max_submaps_maintain);
  LOG(INFO) << "max submap maintain is: "
            << pose_graph_param.max_submaps_maintain;

  LocalizationParam localization_param;
  localization_param.local_slam_param = param;
  localization_param.pose_graph_param = pose_graph_param;
  std::string save_path("");
  this->declare_parameter("map_save_path");
  this->get_parameter("map_save_path", save_path);
  localization_param.pbstream_file_path = save_path + "graph.pbstream";
  this->declare_parameter("thread_num_pool");
  this->get_parameter("thread_num_pool", localization_param.thread_num_pool);
  LOG(INFO) << "thread pool num is: " << localization_param.thread_num_pool;
  localization_.reset(new Localization(localization_param));
  localization_param_ = localization_param;
  bool is_vision_constraint = false;
  this->declare_parameter("is_vision_constraint");
  this->get_parameter("is_vision_constraint", is_vision_constraint);
  if (is_vision_constraint) {
    reloc_client_ =
        this->create_client<cyberdog_visions_interfaces::srv::Reloc>(
            "reloc_service");
  }

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
  auto node_namespace = this->get_namespace();
  bool is_namespace = true;
  if (node_namespace == std::string("/")) {
    is_namespace = false;
  }
  std::string imu_topic("");
  this->declare_parameter("imu_topic", imu_topic);
  this->get_parameter("imu_topic", imu_topic);
  if (is_namespace) {
    imu_topic = node_namespace + imu_topic;
  }
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationNode::ImuCallBack, this, std::placeholders::_1),
      sub_imu_opt);

  std::string odometry_topic("");
  this->declare_parameter("odometry_topic", odometry_topic);
  this->get_parameter("odometry_topic", odometry_topic);
  if (is_namespace) {
    odometry_topic = node_namespace + odometry_topic;
  }
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationNode::OdomCallback, this, std::placeholders::_1),
      sub_odom_opt);
  std::string laser_topic("");
  this->declare_parameter("laser_scan_topic", laser_topic);
  this->get_parameter("laser_scan_topic", laser_topic);
  if (is_namespace) {
    laser_topic = node_namespace + laser_topic;
  }
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationNode::LaserCallBack, this, std::placeholders::_1),
      sub_laser_opt);
  std::vector<double> tmp{0, -1, 0, 0, 0, -1, 1, 0, 0};
  this->declare_parameter("imu_to_odom");
  rclcpp::Parameter tf = this->get_parameter("imu_to_odom");
  tmp = tf.as_double_array();
  transform_ << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7],
      tmp[8];
  std::vector<double> l_t_o{0, 0, 0};
  this->declare_parameter("laser_to_odom");
  rclcpp::Parameter tf1 = this->get_parameter("laser_to_odom");
  l_t_o = tf1.as_double_array();
  Eigen::Vector3d translation;
  translation << l_t_o[0], l_t_o[1], l_t_o[2];
  laser_t_odom_ = transform::Rigid3d::Translation(translation);

  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("laser_pose", 10);
  pc_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
  odom_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  reloc_publisher_ =
      this->create_publisher<std_msgs::msg::Int32>("laser_reloc_result", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  std::string start_location_service_name;
  this->declare_parameter("start_location_service_name");
  this->get_parameter("start_location_service_name",
                      start_location_service_name);

  start_location_service_ = create_service<std_srvs::srv::SetBool>(
      start_location_service_name,
      std::bind(&LocalizationNode::StartLocationCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  std::string stop_location_service_name;
  this->declare_parameter("stop_location_service_name");
  this->get_parameter("stop_location_service_name", stop_location_service_name);
  stop_location_service_ = create_service<std_srvs::srv::SetBool>(
      stop_location_service_name,
      std::bind(&LocalizationNode::StopLocationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  this->declare_parameter("pose_save_path");
  this->get_parameter("pose_save_path", pose_save_path_);
  bool need_save = false;
  this->declare_parameter("need_save_pose");
  this->get_parameter("need_save_pose", need_save);
  if (need_save) {
    pose_recorder_.reset(new PoseRecorder(pose_save_path_));
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationNode::on_activate(
    const rclcpp_lifecycle::State& state) {
  localization_->SetRelocPublisher(reloc_publisher_);
  localization_->SetCallback(
      [this](const transform::Rigid3d& pose, const sensor::RangeData& pc) {
        PosePcCallBack(pose, pc);
      });
  localization_->Initialize();
  if (not reloc_thread_)
    reloc_thread_.reset(
        new std::thread(std::bind(&LocalizationNode::RelocLoop, this)));
  else
    LOG(WARNING) << "Reloc Thread already exist";
  pose_publisher_->on_activate();
  pc_publisher_->on_activate();
  odom_publisher_->on_activate();
  reloc_publisher_->on_activate();
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationNode::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  localization_->Stop();
  localization_.reset(new Localization(localization_param_));
  reloc_thread_ = nullptr;
  pc_publisher_->on_deactivate();
  pose_publisher_->on_deactivate();
  odom_publisher_->on_deactivate();
  reloc_publisher_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationNode::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  localization_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationNode::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  LOG(INFO) << "Shutting Down";
  return nav2_util::CallbackReturn::SUCCESS;
}

void LocalizationNode::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu) {
  if (not is_on_active_status_) return;
  sensor::ImuData imu_meas;
  Eigen::Vector3d acc, gyro;
  acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
      imu->linear_acceleration.z;
  gyro << imu->angular_velocity.x, imu->angular_velocity.y,
      imu->angular_velocity.z;
  imu_meas.linear_acceleration = (acc.transpose() * transform_).transpose();
  imu_meas.angular_velocity = (gyro.transpose() * transform_).transpose();
  imu_meas.time = common::FromUniversal(
      (imu->header.stamp.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (imu->header.stamp.nanosec + 50) / 100);
  localization_->AddImuData(imu_meas);
}

void LocalizationNode::OdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  if (not is_on_active_status_) return;
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
  localization_->AddOdometryData(odom_meas);
}

void LocalizationNode::LaserCallBack(
    const sensor_msgs::msg::LaserScan::SharedPtr laser) {
  if (not is_on_active_status_) return;
  std::vector<sensor::RangefinderPoint> points;
  sensor_msgs::msg::PointCloud2 cloud;
  common::Time time =
      common::FromUniversal((laser->header.stamp.sec +
                             common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                10000000ll +
                            (laser->header.stamp.nanosec + 50) / 100);
  if (last_laser_time_ > common::ToUniversal(time)) {
    LOG(ERROR) << "time back!!!!!!!!!!";
    return;
  }
  last_laser_time_ = common::ToUniversal(time);
  LOG(INFO) << "laser time stamp is: " << common::ToUniversal(time);
  projector_.projectLaser(*laser, cloud, 20.0);
  pcl::PointCloud<pcl::PointXYZ> raw_cloud;
  pcl::fromROSMsg(cloud, raw_cloud);
  for (size_t i = 0; i < raw_cloud.points.size(); ++i) {
    sensor::RangefinderPoint pt;
    pt.position.x() = raw_cloud.points[i].x;
    pt.position.y() = raw_cloud.points[i].y;
    pt.position.z() = raw_cloud.points[i].z;
    sensor::RangefinderPoint transformed_pt =
        sensor::RangefinderPoint{laser_t_odom_.cast<float>() * pt.position};
    points.push_back(transformed_pt);
  }
  sensor::PointCloud pc(time, points);
  if (localization_->state() == State::RELOCATION) {
    // Relocate job add
    {
      std::unique_lock<std::mutex> lk(job_mutex_);
      jobs_.push_back(LoopJob());
      job_condvar_.notify_all();
    }
  }
  localization_->AddRangeData(pc);
}

void LocalizationNode::PosePcCallBack(const transform::Rigid3d& pose,
                                      const sensor::RangeData& pc) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "laser_odom";

  odometry.header.stamp.sec = common::ToRosTime(pc.returns.time()).seconds();
  odometry.header.stamp.nanosec =
      common::ToRosTime(pc.returns.time()).nanoseconds();
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.position.x = pose.translation().x();
  odometry.pose.pose.position.y = pose.translation().y();
  odometry.pose.pose.position.z = pose.translation().z();
  odometry.pose.pose.orientation.x = pose.rotation().x();
  odometry.pose.pose.orientation.y = pose.rotation().y();
  odometry.pose.pose.orientation.z = pose.rotation().z();
  odometry.pose.pose.orientation.w = pose.rotation().w();
  odom_publisher_->publish(odometry);
  geometry_msgs::msg::PoseStamped pose_pub;
  pose_pub.header.frame_id = "laser_odom";
  pose_pub.header.stamp.sec = common::ToRosTime(pc.returns.time()).seconds();
  pose_pub.header.stamp.nanosec =
      common::ToRosTime(pc.returns.time()).nanoseconds();
  pose_pub.pose.orientation.x = pose.rotation().x();
  pose_pub.pose.orientation.y = pose.rotation().y();
  pose_pub.pose.orientation.z = pose.rotation().z();
  pose_pub.pose.orientation.w = pose.rotation().w();
  pose_pub.pose.position.x = pose.translation().x();
  pose_pub.pose.position.y = pose.translation().y();
  pose_pub.pose.position.z = pose.translation().z();
  pose_publisher_->publish(pose_pub);
  auto pc_ros = pc.returns.ToPointCloud2();
  pc_ros.header.frame_id = "laser_odom";
  pc_publisher_->publish(pc_ros);

  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  LOG(INFO) << "time stamp: " << common::ToUniversal(pc.returns.time());
  if (pose_recorder_ != nullptr) {
    pose_recorder_->InputRecordPose(common::ToUniversal(pc.returns.time()),
                                    pose.rotation(), pose.translation());
  }
  t.header.stamp.sec = common::ToRosTime(pc.returns.time()).seconds();
  t.header.stamp.nanosec = common::ToRosTime(pc.returns.time()).nanoseconds();
  t.header.frame_id = "vodom";
  t.child_frame_id = "base_link";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = pose.translation().x();
  t.transform.translation.y = pose.translation().y();
  t.transform.translation.z = 0.0;

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message

  t.transform.rotation.x = pose.rotation().x();
  t.transform.rotation.y = pose.rotation().y();
  t.transform.rotation.z = pose.rotation().z();
  t.transform.rotation.w = pose.rotation().w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}

void LocalizationNode::StartLocationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    LOG(ERROR) << "Recieved Trigger Location request But not in active "
                  "state, Ignoring!!!";
    return;
  }
  is_on_active_status_ = request->data;
  response->success = true;
}

void LocalizationNode::StopLocationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    LOG(ERROR) << "Recieved Stop Location request But not in active state, "
                  "Ignoring!!!!";
    return;
  }
  is_on_active_status_ = false;
  if (reloc_thread_ && reloc_thread_->joinable()) {
    reloc_thread_->join();
  }
  if (pose_recorder_ != nullptr) {
    pose_recorder_->WriteRecordPose();
    pose_recorder_->Close();
  }
  bool success = localization_->Stop();
  response->success = success;
}

bool LocalizationNode::Request(RelocPose* pose) {
  auto request =
      std::make_shared<cyberdog_visions_interfaces::srv::Reloc::Request>();
  request->reloc_id = reloc_id_;
  while (!reloc_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      LOG(ERROR) << "Interrupted while waiting for the service, Exiting";
      return false;
    }
    LOG(INFO) << "Service not available, waiting again";
  }
  auto result = reloc_client_->async_send_request(request);
  // auto status = result.wait_for(5s);
  sleep(5);
  LOG(INFO) << "wait ended";
  RelocPose pose_result;
  if (result.valid()) {
    auto r_pose = result.get()->pose;
    Eigen::Quaterniond rotation;
    rotation.x() = r_pose.pose.orientation.x;
    rotation.y() = r_pose.pose.orientation.y;
    rotation.z() = r_pose.pose.orientation.z;
    rotation.w() = r_pose.pose.orientation.w;
    Eigen::Vector3d translation;
    translation.x() = r_pose.pose.position.x;
    translation.y() = r_pose.pose.position.y;
    translation.z() = r_pose.pose.position.z;
    pose_result.pose.Rotation(rotation);
    pose_result.pose.Translation(translation);
    pose_result.time = common::Time::max();
    LOG(INFO) << "get reloc pose" << pose_result.pose.DebugString();
    *pose = pose_result;
  } else {
    LOG(ERROR) << "Failed to get response";
    return false;
  }
  return true;
}

void LocalizationNode::RelocLoop() {
  // Reloc loop from vision
  while (true) {
    if (not is_on_active_status_) {
      usleep(1000);
    } else if (reloc_client_ == nullptr) {
      if (localization_->state() == State::RELOCATION ||
          localization_->state() == State::RELOCATING) {
        RelocPose pose;
        pose.pose = transform::Rigid3d::Identity();
        pose.time = common::Time::max();
        auto reloc_pose = std::make_shared<RelocPose>(pose);
        localization_->GetInitialPose(reloc_pose);
      } else {
        usleep(1000);
      }
      if (!is_on_active_status_) return;
    } else {
      LoopJob job;
      {
        std::unique_lock<std::mutex> lk(job_mutex_);
        while (jobs_.empty() && is_on_active_status_) job_condvar_.wait(lk);

        if (!is_on_active_status_) return;

        job = jobs_.front();

        jobs_.pop_front();
      }
      if (job.type == LoopJob::INIT_LOCATION) {
        RelocPose pose_init;
        bool success = Request(&pose_init);
        if (success) {
          auto reloc_pose = std::make_shared<RelocPose>(pose_init);
          localization_->GetInitialPose(reloc_pose);
        }
      }
      if (job.type == LoopJob::NORMAL_LOCATION) {
        // Norm find constraint by vision
      }
    }
  }
}

}  // namespace laser_slam
}  // namespace cartographer
