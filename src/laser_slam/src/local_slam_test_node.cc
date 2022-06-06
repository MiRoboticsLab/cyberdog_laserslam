/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <memory>
#include <thread>
#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laser_slam/local_slam.h"
#include "laser_slam/back_end.h"

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
class LaserSlamTestNode : public rclcpp::Node {
 public:
  LaserSlamTestNode()
      : Node("laser_slam_test_node"),
        process_thread_(nullptr),
        first_(true),
        first_frame_(true),
        start_(false) {
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
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserSlamTestNode::LaserCallBack, this,
                      std::placeholders::_1),
            sub_laser_opt);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
    pc2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud2", 10);
    grid_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", 10);
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mi1045856/camera/imu", 10,
        std::bind(&LaserSlamTestNode::ImuCallback, this, std::placeholders::_1),
        sub_imu_opt);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mi1045856/odom_out", 10,
        std::bind(&LaserSlamTestNode::OdomCallback, this,
                  std::placeholders::_1),
        sub_odom_opt);
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "laser_pose", 10);
    LocalSlamParam param;
    param.use_imu = true;
    param.use_real_time_correlative_scan_matching = true;
    param.num_accumulated = 1;
    param.use_filter = false;
    param.gravity_constant = 10.0;
    param.pose_duration_time = 0.01;
    param.max_time_seconds = 5.0;
    param.max_distance_meters = 0.2;
    param.max_angle_radians = M_PI / 180;
    param.min_range = 0.3;
    param.max_range = 15.0;
    param.missing_data_ray_length = 5.0;
    param.voxel_filter_size = 0.025;
    param.min_num_points = 200;
    param.max_length = 0.5;
    param.max_range_scale = 20.0;
    param.submap_param.grid_insert_type = 0;
    param.submap_param.grid_type = 0;
    param.submap_param.num_range_data = 20;
    param.submap_param.resolution = 0.05;
    param.submap_param.probability_insert_param.hit_probability = 0.55;
    param.submap_param.probability_insert_param.miss_probability = 0.49;
    param.submap_param.probability_insert_param.insert_free = true;
    param.ceres_param.max_num_iterations = 20;
    param.ceres_param.num_threads = 1;
    param.ceres_param.occupied_space_weight = 1.0;
    param.ceres_param.rotation_weight = 1.0;
    param.ceres_param.translation_weight = 1.0;
    param.ceres_param.use_nonmonotonic_steps = false;
    param.real_time_param.angular_search_window = (M_PI / 180) * 20.0;
    param.real_time_param.linear_search_window = 0.1;
    param.real_time_param.rotation_delta_cost_weight = 1e-1;
    param.real_time_param.translation_delta_cost_weight = 1e-1;
    BackEndParam back_end_param;
    back_end_param.local_slam_param = param;
    back_end_param.thread_num_pool = 4;
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
    pose_graph_param.optimize_every_n_nodes = 90;
    pose_graph_param.max_num_final_iterations = 200;
    pose_graph_param.matcher_rotation_weight = 5e2;
    pose_graph_param.matcher_translation_weight = 1.6e3;
    back_end_param.pose_graph_param = pose_graph_param;
    back_end_.reset(new BackEnd(0, back_end_param));
    local_slam_.reset(new LocalSlam(param));
    inserter_ = std::make_unique<mapping::ProbabilityGridRangeDataInserter2D>(
        param.submap_param.probability_insert_param);
    // inserter_.reset(mapping::ProbabilityGridRangeDataInserter2D(
    //     param.submap_param.probability_insert_param));
    grid_ = std::make_unique<mapping::ProbabilityGrid>(
        mapping::MapLimits(0.05,
                           Eigen::Vector2d::Zero() +
                               0.5 * 100 * 0.05 * Eigen::Vector2d::Ones(),
                           mapping::CellLimits(100, 100)),
        &conversion_tables_);
    transform_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  }

  virtual ~LaserSlamTestNode() {}

  bool Start() {
    start_ = true;
    return true;
  }

  bool Stop() {
    start_ = false;
    // if (process_thread_ && process_thread_->joinable()) {
    //   process_thread_->join();
    // }
    LOG(INFO) << "fjskd";
    // std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid;
    // rclcpp::Time time;
    // grid = grid_->ToRosOccupancyMsg(0.05, "laser", time);
    // LOG(INFO) << "grid size is: " << grid->info.width << " , "
    //           << grid->info.height;
    // while (1) {
    //   grid_publisher_->publish(*grid);
    // }
    return true;
  }

 private:
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
    std::vector<sensor::RangefinderPoint> points;
    sensor_msgs::msg::PointCloud2 cloud;
    common::Time time = common::FromRosTime(laser->header.stamp);
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
      points.push_back(pt);
    }
    if (back_end_->LatestPoseExtrapolatorTime() == common::Time::min() ||
        time < back_end_->LatestPoseExtrapolatorTime()) {
      LOG(INFO) << "return cause by time"
                << back_end_->LatestPoseExtrapolatorTime();
      return;
    }
    sensor::PointCloud pc(time, points);
    std::unique_ptr<laser_slam::TmpResult> tmp_result =
        back_end_->AddRangeData(pc);
    if (tmp_result != nullptr) {
      geometry_msgs::msg::PoseStamped pose_pub;
      pose_pub.header.frame_id = "laser";
      pose_pub.header.stamp = laser->header.stamp;
      pose_pub.pose.orientation.x = tmp_result->local_pose.rotation().x();
      pose_pub.pose.orientation.y = tmp_result->local_pose.rotation().y();
      pose_pub.pose.orientation.z = tmp_result->local_pose.rotation().z();
      pose_pub.pose.orientation.w = tmp_result->local_pose.rotation().w();
      pose_pub.pose.position.x = tmp_result->local_pose.translation().x();
      pose_pub.pose.position.y = tmp_result->local_pose.translation().y();
      pose_pub.pose.position.z = tmp_result->local_pose.translation().z();
      publisher_->publish(pose_pub);
      inserter_->Insert(tmp_result->local_range_data, grid_.get());
      grid_publisher_->publish(*grid_->ToRosOccupancyMsg(
          0.05, "laser_odom", rclcpp::Time::max(), false, ""));
    }
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
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
    // local_slam_->AddImuData(imu_meas);
    back_end_->AddImuData(imu_meas);
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
    // local_slam_->AddOdometryData(odom_meas);
    back_end_->AddOdometryData(odom_meas);
  }

  void Process() {
    while (start_) {
      usleep(1000);
      // std::unique_ptr<nav_msgs::msg::OccupancyGrid> pub_grid;
      // {
      //   std::lock_guard<std::mutex> lk(range_data_lk_);
      //   rclcpp::Time time;
      //   pub_grid = grid_->ToRosOccupancyMsg(0.05, "laser", time);
      // }
      // grid_publisher_->publish(*pub_grid);
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_odometry_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_imu_subscriber_;
  rclcpp::CallbackGroup::SharedPtr callback_laser_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::mutex range_data_lk_;
  std::shared_ptr<std::thread> process_thread_;
  LocalSlamPtr local_slam_;
  Eigen::Matrix3d transform_;
  laser_geometry::LaserProjection projector_;
  tf2::BufferCore buffer_core_;
  bool first_;
  bool first_frame_;
  sensor::PointCloud last_pc_;
  common::Time last_time_;
  std::deque<sensor::PointCloud> pcs_;
  bool start_;
  std::unique_ptr<mapping::RangeDataInserterInterface> inserter_;
  std::shared_ptr<BackEnd> back_end_;
  std::unique_ptr<mapping::ProbabilityGrid> grid_;
  mapping::ValueConversionTables conversion_tables_;
};

}  // namespace laser_slam
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<cartographer::laser_slam::LaserSlamTestNode> local_slam =
      std::make_shared<cartographer::laser_slam::LaserSlamTestNode>();
  local_slam->Start();
  executor.add_node(local_slam);
  executor.spin();

  rclcpp::shutdown();
  local_slam->Stop();
  return 0;
}
