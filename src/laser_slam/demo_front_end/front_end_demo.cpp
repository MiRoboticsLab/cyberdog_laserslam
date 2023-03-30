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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <map>

#include "common/thread_pool.h"
#include "pose_graph/bundle_adjustment.h"
#include "protos/proto_stream.h"
#include "protos/proto_stream_interface.h"
#include "rviz_display/probability_grid_conversion.h"
#include "laser_slam/base_data/grid_for_navigation.hpp"
#include "laser_slam/base_data/pose_recorder.hpp"
#include "laser_slam/base_data/submap_points_batch.hpp"
#include "laser_slam/final_map_generator.hpp"
#include "laser_slam/local_slam.hpp"

#include "tf2/buffer_core.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization/msg/matching_result.hpp"
using namespace std::chrono_literals;
namespace cartographer
{
namespace laser_slam
{
class FrontEndDemo : public rclcpp::Node
{
public:
  FrontEndDemo()
  : Node("demo_front_end_node"), local_slam_(nullptr), frame_id_(""),
    map_save_path_(""), map_generator_(nullptr), thread_pool_(4),
    pose_graph_(nullptr), pose_recorder_(nullptr),
    map_process_thread_(nullptr) {}

  virtual ~FrontEndDemo() {}

  bool Initialization()
  {
    // Get parameters from yaml
    LocalSlamParam param;
    std::string frame_id("");
    this->declare_parameter("frame_id", frame_id);
    this->get_parameter("frame_id", frame_id);
    frame_id_ = frame_id;
    this->declare_parameter("use_imu");
    this->get_parameter("use_imu", param.use_imu);
    this->declare_parameter("use_real_time_correlative_scan_matching");
    this->get_parameter(
      "use_real_time_correlative_scan_matching",
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
    this->get_parameter(
      "missing_data_ray_length",
      param.missing_data_ray_length);
    this->declare_parameter("voxel_filter_size");
    this->get_parameter("voxel_filter_size", param.voxel_filter_size);
    this->declare_parameter("min_num_points");
    this->get_parameter("min_num_points", param.min_num_points);
    this->declare_parameter("max_length");
    this->get_parameter("max_length", param.max_length);
    this->declare_parameter("max_range_scale");
    this->get_parameter("max_range_scale", param.max_range_scale);
    this->declare_parameter("submap_param.grid_insert_type");
    this->get_parameter(
      "submap_param.grid_insert_type",
      param.submap_param.grid_insert_type);
    this->declare_parameter("submap_param.num_range_data");
    this->get_parameter(
      "submap_param.num_range_data",
      param.submap_param.num_range_data);
    this->declare_parameter("submap_param.grid_type");
    this->get_parameter(
      "submap_param.grid_type",
      param.submap_param.grid_type);
    this->declare_parameter("submap_param.resolution");
    this->get_parameter(
      "submap_param.resolution",
      param.submap_param.resolution);
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
    this->get_parameter(
      "submap_param.probability_inserter_param.insert_free",
      param.submap_param.probability_insert_param.insert_free);
    this->declare_parameter(
      "ceres_scan_matching_param.use_nonmonotonic_steps");
    this->get_parameter(
      "ceres_scan_matching_param.use_nonmonotonic_steps",
      param.ceres_param.use_nonmonotonic_steps);
    this->declare_parameter("ceres_scan_matching_param.max_num_iterations");
    this->get_parameter(
      "ceres_scan_matching_param.max_num_iterations",
      param.ceres_param.max_num_iterations);
    this->declare_parameter("ceres_scan_matching_param.num_threads");
    this->get_parameter(
      "ceres_scan_matching_param.num_threads",
      param.ceres_param.num_threads);
    this->declare_parameter(
      "ceres_scan_matching_param.occupied_space_weight");
    this->get_parameter(
      "ceres_scan_matching_param.occupied_space_weight",
      param.ceres_param.occupied_space_weight);
    this->declare_parameter("ceres_scan_matching_param.translation_weight");
    this->get_parameter(
      "ceres_scan_matching_param.translation_weight",
      param.ceres_param.translation_weight);
    this->declare_parameter("ceres_scan_matching_param.rotation_weight");
    this->get_parameter(
      "ceres_scan_matching_param.rotation_weight",
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
      "real_time_correlative_scan_matching_param."
      "rotation_delta_cost_weight");
    this->get_parameter(
      "real_time_correlative_scan_matching_param."
      "rotation_delta_cost_weight",
      param.real_time_param.rotation_delta_cost_weight);
    local_slam_.reset(new LocalSlam(param));

    ConstraintBuilderParam constraint_param;
    constraint_param.min_score = 0.55;
    constraint_param.log_matches = true;
    constraint_param.loop_closure_rotation_weight = 1.e5;
    constraint_param.loop_closure_translation_weight = 1.1e4;
    constraint_param.max_constraint_distance = 2.;
    constraint_param.ratio = 0.3;
    constraint_param.ceres_param.max_num_iterations = 10;
    constraint_param.ceres_param.num_threads = 1;
    constraint_param.ceres_param.use_nonmonotonic_steps = true;
    constraint_param.ceres_param.occupied_space_weight = 20.;
    constraint_param.ceres_param.translation_weight = 10.;
    constraint_param.ceres_param.rotation_weight = 1.;
    constraint_param.fast_param.branch_and_bound_depth = 7;
    constraint_param.fast_param.angular_search_window = (M_PI / 180) * 15;
    constraint_param.fast_param.linear_search_window = 7.;
    PoseGraph2DParam pose_graph_param;
    pose_graph_param.constraint_builder_param = constraint_param;
    OptimizationParam optimization_param;
    optimization_param.huber_scale = 1e1;
    optimization_param.max_num_iterations = 50;
    optimization_param.num_threads = 7;
    optimization_param.report_full_summary = true;
    pose_graph_param.optimization_param = optimization_param;
    pose_graph_param.optimize_every_n_nodes = 10;
    pose_graph_param.max_num_final_iterations = 200;
    pose_graph_param.matcher_rotation_weight = 5e2;
    pose_graph_param.matcher_translation_weight = 1.6e3;
    pose_graph_param.max_submaps_maintain = 0;
    pose_graph_.reset(
      new pose_graph::optimization::BundleAdjustment(
        pose_graph_param, &thread_pool_));
    pose_graph_->SetSubmapCallback(
      [this](const mapping::SubmapId & id,
      const std::shared_ptr<const mapping::Submap> & data) {
        SubmapTest(id, data);
      });
    // subscribe and publisher initialization
    pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "laser_pose", 10);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10);
    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "intensity_scan", 10);
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map_test", 10);

    callback_imu_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_laser_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_odometry_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
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
      std::bind(&FrontEndDemo::ImuCallBack, this, std::placeholders::_1),
      sub_imu_opt);
    std::string odometry_topic("");
    this->declare_parameter("odometry_topic", odometry_topic);
    this->get_parameter("odometry_topic", odometry_topic);
    LOG(INFO) << odometry_topic;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&FrontEndDemo::OdomCallback, this, std::placeholders::_1),
      sub_odom_opt);
    std::string laser_topic("");
    this->declare_parameter("laser_scan_topic", laser_topic);
    this->get_parameter("laser_scan_topic", laser_topic);
    laser_subscription_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, rclcpp::SensorDataQoS(),
      std::bind(
        &FrontEndDemo::LaserCallBack, this,
        std::placeholders::_1),
      sub_laser_opt);

    std::vector<double> tmp{0, -1, 0, 0, 0, -1, 1, 0, 0};
    this->declare_parameter("imu_to_odom");
    rclcpp::Parameter tf = this->get_parameter("imu_to_odom");
    tmp = tf.as_double_array();
    transform_ << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6],
      tmp[7], tmp[8];
    std::vector<double> l_t_o{0, 0, 0};
    this->declare_parameter("laser_to_odom");
    rclcpp::Parameter tf1 = this->get_parameter("laser_to_odom");
    l_t_o = tf1.as_double_array();
    Eigen::Vector3d translation;
    translation << l_t_o[0], l_t_o[1], l_t_o[2];
    laser_t_odom_ = transform::Rigid3d::Translation(translation);
    this->declare_parameter("map_save_path");
    this->get_parameter("map_save_path", map_save_path_);
    std::string pose_save_path = map_save_path_ + "pose.txt";
    pose_recorder_.reset(new PoseRecorder(pose_save_path));
    map_generator_.reset(
      new FinalMapGenerator(param.submap_param.probability_insert_param));
    grid_.reset(
      new GridForNavigation(
        0.05, param.submap_param.probability_insert_param));
    start_ = true;
    // if (not map_process_thread_) {
    //   map_process_thread_.reset(
    //       new std::thread(std::bind(&FrontEndDemo::SubmapPublish,
    //       this)));
    // }

    map_grow_.reset(new SubmapPointsBatch(0.05, 1000, 1000));
    map_grow_->StartThread();
    grid_publish_timer_ = create_wall_timer(
      500ms, std::bind(&FrontEndDemo::SubmapPublish, this));
    return true;
  }

  bool Stop()
  {
    start_ = false;
    // if (map_process_thread_ && map_process_thread_->joinable()) {
    //   map_process_thread_->join();
    // }
    map_grow_->QuitThread();
    pose_graph_->RunFinalOptimization();
    protos::mapping::proto::PoseGraphHeader header;
    header.set_format_version(1);
    protos::mapping::proto::PoseGraph graph_proto =
      pose_graph_->ToProto(0, true);
    std::string file_name = map_save_path_ + "graph.pbstream";
    stream::ProtoStreamWriter writer(file_name);
    writer.WriteProto(header);
    writer.WriteProto(graph_proto);
    CHECK(writer.Close());
    const auto pose_graph_data = pose_graph_->pose_graph_data();
    std::vector<sensor::RangeData> range_datas;
    auto begin_it = pose_graph_data.trajectory_nodes.BeginOfTrajectory(0);
    auto end_it = pose_graph_data.trajectory_nodes.EndOfTrajectory(0);
    for (; begin_it != end_it; ++begin_it) {
      transform::Rigid3d local_to_global =
        pose_graph_data.trajectory_nodes.at(begin_it->id).global_pose *
        pose_graph_data.trajectory_nodes.at(begin_it->id)
        .constant_data->local_pose.inverse();
      auto range_data = id_data_.at(begin_it->id);
      LOG(INFO) << "delta pose is: " << local_to_global.DebugString() <<
        "id is: " << begin_it->id.node_index;
      auto pc = sensor::TransformRangeData(
        range_data,
        local_to_global.cast<float>());
      range_datas.push_back(pc);
    }
    grid_->RayCastByProbability(range_datas);
    std::string map_name = map_save_path_ + "map";
    grid_->WritePgmByProbabilityGrid(map_name);
    pose_recorder_->Write(pose_graph_data.trajectory_nodes);
    pose_recorder_->Close();
  }

private:
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu)
  {
    sensor::ImuData imu_meas;
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
      imu->linear_acceleration.z;
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
      imu->angular_velocity.z;
    imu_meas.linear_acceleration =
      (acc.transpose() * transform_).transpose();
    imu_meas.angular_velocity = (gyro.transpose() * transform_).transpose();
    imu_meas.time = common::FromUniversal(
      (imu->header.stamp.sec +
      common::kUtsEpochOffsetFromUnixEpochInSeconds) *
      10000000ll +
      (imu->header.stamp.nanosec + 50) / 100);
    local_slam_->AddImuData(imu_meas);
  }
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
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
      (odom->header.stamp.sec +
      common::kUtsEpochOffsetFromUnixEpochInSeconds) *
      10000000ll +
      (odom->header.stamp.nanosec + 50) / 100);
    local_slam_->AddOdometryData(odom_meas);
  }
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laser)
  {
    std::vector<sensor::RangefinderPoint> points;
    sensor_msgs::msg::PointCloud2 cloud;
    common::Time time = common::FromUniversal(
      (laser->header.stamp.sec +
      common::kUtsEpochOffsetFromUnixEpochInSeconds) *
      10000000ll +
      (laser->header.stamp.nanosec + 50) / 100);
    LOG(INFO) << "delta time is:: " << common::ToSeconds(time - last_time_);
    last_time_ = time;
    projector_.projectLaser(*laser, cloud, 20.0);

    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(cloud, raw_cloud);
    for (size_t i = 0; i < raw_cloud.points.size(); ++i) {
      sensor::RangefinderPoint pt;
      pt.position.x() = raw_cloud.points[i].x;
      pt.position.y() = raw_cloud.points[i].y;
      pt.position.z() = raw_cloud.points[i].z;
      sensor::RangefinderPoint transformed_pt = sensor::RangefinderPoint{
        laser_t_odom_.cast<float>() * pt.position};
      // to_do: transform pt to tracking frame
      points.push_back(transformed_pt);
    }
    if (local_slam_->LatestPoseExtrapolatorTime() == common::Time::min() ||
      time < local_slam_->LatestPoseExtrapolatorTime())
    {
      LOG(INFO) << "return cause by time" <<
        local_slam_->LatestPoseExtrapolatorTime();
      return;
    }
    sensor::PointCloud pc(time, points);
    std::unique_ptr<MatchingResult> tmp_result =
      local_slam_->AddRangeData(pc);
    if (tmp_result != nullptr) {
      if (tmp_result->insertion_result != nullptr) {
        const auto node =
          tmp_result->insertion_result->node_constant_data;
        const auto submaps =
          tmp_result->insertion_result->insertion_submaps;
        mapping::NodeId node_id =
          pose_graph_->AddNode(node, 0, submaps);
        id_data_[node_id] = tmp_result->range_data_in_local;
      }
      geometry_msgs::msg::PoseStamped pose_pub;
      pose_pub.header.frame_id = frame_id_;
      pose_pub.header.stamp = laser->header.stamp;
      pose_pub.pose.orientation.x = tmp_result->local_pose.rotation().x();
      pose_pub.pose.orientation.y = tmp_result->local_pose.rotation().y();
      pose_pub.pose.orientation.z = tmp_result->local_pose.rotation().z();
      pose_pub.pose.orientation.w = tmp_result->local_pose.rotation().w();
      pose_pub.pose.position.x = tmp_result->local_pose.translation().x();
      pose_pub.pose.position.y = tmp_result->local_pose.translation().y();
      pose_pub.pose.position.z = tmp_result->local_pose.translation().z();
      pose_publisher_->publish(pose_pub);
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl_cloud.points.resize(
        tmp_result->range_data_in_local.returns.size());
      for (size_t j = 0;
        j < tmp_result->range_data_in_local.returns.size(); ++j)
      {
        pcl_cloud.points[j].x =
          tmp_result->range_data_in_local.returns[j].position.x();
        pcl_cloud.points[j].y =
          tmp_result->range_data_in_local.returns[j].position.y();
        pcl_cloud.points[j].z =
          tmp_result->range_data_in_local.returns[j].position.z();
      }

      sensor_msgs::msg::PointCloud2 pub_pc;
      pcl::toROSMsg(pcl_cloud, pub_pc);
      pub_pc.header.frame_id = frame_id_;
      pc_publisher_->publish(pub_pc);
      sensor::RangeData range = tmp_result->range_data_in_local;
      range_datas_.push_back(range);
    }
  }

  void SubmapTest(
    const mapping::SubmapId & id,
    const std::shared_ptr<const mapping::Submap> & data)
  {
    if (!pub_) {
      pub_ = true;
    }
    map_grow_->AddSubmap(id, data);
  }

  void SubmapPublish()
  {
    if (pub_) {
      auto map = map_grow_->ros_grid();
      map.header.frame_id = "laser_odom";
      map_publisher_->publish(map);
    }
  }
  bool start_ = false;
  bool pub_ = false;
  int count_ = 0;
  std::deque<std::shared_ptr<const mapping::Submap>> maps_;
  std::vector<nav_msgs::msg::OccupancyGrid> ros_maps_;
  std::mutex map_lk_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_subscription_;
  rclcpp::TimerBase::SharedPtr grid_publish_timer_;
  LocalSlamPtr local_slam_;
  std::string frame_id_;
  std::string map_save_path_;
  common::Time last_time_;
  Eigen::Matrix3d transform_;
  transform::Rigid3d laser_t_odom_;
  laser_geometry::LaserProjection projector_;
  FinalMapGeneratorPtr map_generator_;
  GridForNavigationPtr grid_;
  std::vector<sensor::RangeData> range_datas_;
  common::ThreadPool thread_pool_;
  pose_graph::optimization::BundleAdjustmentPtr pose_graph_;
  std::map<mapping::NodeId, sensor::RangeData> id_data_;
  std::shared_ptr<PoseRecorder> pose_recorder_;
  std::shared_ptr<std::thread> map_process_thread_;
  SubmapPointsBatchPtr map_grow_;
};
}  // namespace laser_slam

}  // namespace cartographer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::FrontEndDemo> local_slam =
    std::make_shared<cartographer::laser_slam::FrontEndDemo>();
  local_slam->Initialization();
  executor.add_node(local_slam);
  executor.spin();
  local_slam->Stop();

  rclcpp::shutdown();
  return 0;
}
