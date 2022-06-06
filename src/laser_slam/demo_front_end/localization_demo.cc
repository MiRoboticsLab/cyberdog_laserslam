/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laser_slam/localization.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2/buffer_core.h"

namespace cartographer {
namespace laser_slam {
class LocalizationDemo : public rclcpp::Node {
 public:
  LocalizationDemo() : Node("localization"), localization_(nullptr) {}
  virtual ~LocalizationDemo() {}

  bool Initialization() {
    // Get parameters from yaml
    LocalSlamParam param;
    std::string frame_id("");
    this->declare_parameter("frame_id", frame_id);
    this->get_parameter("frame_id", frame_id);
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
    this->get_parameter("missing_data_ray_length",
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
    this->get_parameter("submap_param.grid_insert_type",
                        param.submap_param.grid_insert_type);
    this->declare_parameter("submap_param.num_range_data");
    this->get_parameter("submap_param.num_range_data",
                        param.submap_param.num_range_data);
    this->declare_parameter("submap_param.grid_type");
    this->get_parameter("submap_param.grid_type", param.submap_param.grid_type);
    this->declare_parameter("submap_param.resolution");
    this->get_parameter("submap_param.resolution",
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
    constraint_param.min_score = 0.63;
    constraint_param.reloc_min_score = 0.60;
    constraint_param.log_matches = true;
    constraint_param.loop_closure_rotation_weight = 1.e5;
    constraint_param.loop_closure_translation_weight = 1.1e4;
    constraint_param.max_constraint_distance = 5.;
    constraint_param.max_find_reloc_constraint_distance = 10.;
    constraint_param.ratio = 0.1;
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
    pose_graph_param.optimize_every_n_nodes = 20;
    pose_graph_param.max_num_final_iterations = 200;
    pose_graph_param.matcher_rotation_weight = 5e2;
    pose_graph_param.matcher_translation_weight = 1.6e3;
    pose_graph_param.max_submaps_maintain = 3;
    LocalizationParam localization_param;
    localization_param.local_slam_param = param;
    localization_param.pose_graph_param = pose_graph_param;
    localization_param.channel_name = "localhost:50051";
    std::string save_path("");
    this->declare_parameter("map_save_path");
    this->get_parameter("map_save_path", save_path);
    localization_param.pbstream_file_path = save_path + "graph.pbstream";
    localization_param.thread_num_pool = 4;
    localization_.reset(new Localization(localization_param));
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
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationDemo::ImuCallBack, this, std::placeholders::_1),
        sub_imu_opt);
    std::string odometry_topic("");
    this->declare_parameter("odometry_topic", odometry_topic);
    this->get_parameter("odometry_topic", odometry_topic);
    LOG(INFO) << odometry_topic;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationDemo::OdomCallback, this, std::placeholders::_1),
        sub_odom_opt);
    std::string laser_topic("");
    this->declare_parameter("laser_scan_topic", laser_topic);
    this->get_parameter("laser_scan_topic", laser_topic);
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationDemo::LaserCallBack, this,
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
    LOG(INFO) << "laser to odom is: " << laser_t_odom_.DebugString();
    localization_->Initialize();
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "laser_pose", 10);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    localization_->SetCallback(
        [this](const transform::Rigid3d& pose, const sensor::RangeData& pc) {
          PosePcCallBack(pose, pc);
        });
    return true;
  }

  bool Stop() { return localization_->Stop(); }

 private:
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu) {
    sensor::ImuData imu_meas;
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
        imu->linear_acceleration.z;
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
        imu->angular_velocity.z;
    imu_meas.linear_acceleration = (acc.transpose() * transform_).transpose();
    imu_meas.angular_velocity = (gyro.transpose() * transform_).transpose();
    imu_meas.time =
        common::FromUniversal((imu->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (imu->header.stamp.nanosec + 50) / 100);
    localization_->AddImuData(imu_meas);
  }
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    Eigen::Vector3d position;
    Eigen::Quaterniond angular;
    position << odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0;
    angular = Eigen::Quaterniond(
        odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    sensor::OdometryData odom_meas;
    transform::Rigid3d pose(position, angular);
    odom_meas.pose = pose;
    odom_meas.time =
        common::FromUniversal((odom->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (odom->header.stamp.nanosec + 50) / 100);
    localization_->AddOdometryData(odom_meas);
  }
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
    std::vector<sensor::RangefinderPoint> points;
    sensor_msgs::msg::PointCloud2 cloud;
    common::Time time =
        common::FromUniversal((laser->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (laser->header.stamp.nanosec + 50) / 100);
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
    localization_->AddRangeData(pc);
  }

  void PosePcCallBack(const transform::Rigid3d& pose,
                      const sensor::RangeData& pc) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "laser_odom";
    odometry.header.stamp = common::ToRosTime(pc.returns.time());
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
    pose_pub.header.stamp = common::ToRosTime(pc.returns.time());
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
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  LocalizationPtr localization_;
  laser_geometry::LaserProjection projector_;
  Eigen::Matrix3d transform_;
  transform::Rigid3d laser_t_odom_;
};

}  // namespace laser_slam
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::LocalizationDemo> local_slam =
      std::make_shared<cartographer::laser_slam::LocalizationDemo>();
  local_slam->Initialization();
  executor.add_node(local_slam);
  executor.spin();
  rclcpp::shutdown();

  local_slam->Stop();
  return 0;
}
