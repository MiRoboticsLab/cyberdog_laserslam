/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <iostream>
#include <iomanip>
#include <fstream>

#include <glog/logging.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "range_data_matching/filter/imu_integration_interface.h"
#include "range_data_matching/filter/imu_integration_midpoint.h"
#include "range_data_matching/line_segmentation/transform_pc_2_Image.h"
#include "range_data_matching/line_segmentation/line_extraction.h"
constexpr double kNanosecToSec = 1e-9;
namespace cartographer {
namespace mapping {
class ImuIntegrationTest : public rclcpp::Node {
 public:
  ImuIntegrationTest() : Node("integrator"), first_update_(true) {
    delta_p_.setZero();
    delta_q_.setIdentity();
    delta_v_.setZero();
    linear_bg_.setZero();
    linear_ba_.setZero();
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mi0016548/camera/imu", 10,
        std::bind(&ImuIntegrationTest::ImuCallBack, this,
                  std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mi0016548/odom_out", 10,
        std::bind(&ImuIntegrationTest::OdomCallBack, this,
                  std::placeholders::_1));
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ImuIntegrationTest::LaserCallBack, this,
                      std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "imu_odom_rviz", 10);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
    V3D gravity;
    gravity << 0.0, 0.0, -kGravity;
    Eigen::Matrix<double, kStateDim, 1> diagonal_covariance;
    diagonal_covariance << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1;
    MXD covariance;
    covariance.resize(kStateDim, kStateDim);
    covariance = diagonal_covariance.asDiagonal();
    Eigen::Matrix<double, 12, 1> diagonal_noise;
    diagonal_noise << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
        0.01, 0.01, 0.01;
    MXD noise;
    noise.resize(12, 12);
    noise = diagonal_noise.asDiagonal();
    transform_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    integrator_.reset(new ImuIntegrationMidPoint(covariance, noise, gravity));
  }
  virtual ~ImuIntegrationTest() {}

 private:
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcs;
    int nums = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    std::vector<Eigen::Vector2d> points;
    std::vector<double> ranges;
    points.resize(nums);
    ranges.resize(nums);
    int real_num = 0;
    for (int i = 0; i < nums; ++i) {
      if (fabs(scan->ranges[i]) > 0.3) {
        ++real_num;
        Eigen::Vector2d pt;
        pt.x() =
            sin(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];
        pt.y() =
            cos(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];
        ranges.push_back(scan->ranges[i]);
        points.push_back(pt);
      }
    }
    std::vector<double> new_ranges;
    std::vector<Eigen::Vector2d> new_points;
    LOG(INFO) << "vector capacity : " << new_ranges.capacity();
    for (int j = 0; j < ranges.size(); ++j) {
      if (fabs(ranges[j]) > 0.3) {
        LOG(INFO) << "ranges go: " << ranges[j];
        new_ranges.push_back(ranges[j]);
        new_points.push_back(points[j]);
      }
    }
    std::ofstream output, output1;
    if (only_one_) {
      LOG(INFO) << "vector capacity : " << new_ranges.capacity();

      for (size_t x = 0; x < new_ranges.size(); ++x) {
        LOG(INFO) << "scan range is: " << new_ranges[x]
                  << "point: " << new_points[x].x();
      }
      double cluster_threshold = 0.15;
      double max_dist_from_line = 0.1;
      double slope_threshold = 0.01;
      int min_points = 10;
      int max_iteration = 25;
      line_extration::LineExtraction extractor(
          cluster_threshold, max_dist_from_line, slope_threshold, min_points,
          max_iteration);
      // line_extration::Lines lines =
      //     extractor.SplitAndMergeWithPolarCoordinate(new_ranges, new_points);
      LOG(INFO) << "extract finished";
      output.open("/home/zfx/pc.txt", std::ios::out | std::ios::trunc);
      output << std::fixed;
      for (size_t x = 0; x < new_points.size(); ++x) {
        output << std::setprecision(8) << new_points[x].x() << "  "
               << new_points[x].y() << std::endl;
      }
      output.close();
      // output1.open("/home/zfx/lines.txt", std::ios::out | std::ios::trunc);
      // for (size_t y = 0; y < lines.size(); ++y) {
      //   output1 << std::setprecision(8) << lines[y].a.x() << "  "
      //           << lines[y].a.y() << "  " << lines[y].b.x() << "  "
      //           << lines[y].b.y() << std::endl;
      // }
      // output1.close();
      only_one_ = false;
    }
  }
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu) {
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
        imu->linear_acceleration.z;
    LOG(INFO) << "original acc is: " << acc.transpose();
    acc = (acc.transpose() * transform_).transpose();
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
        imu->angular_velocity.z;
    gyro = (gyro.transpose() * transform_).transpose();
    double timestamp =
        imu->header.stamp.sec + imu->header.stamp.nanosec * kNanosecToSec;
    Eigen::Vector3d result_delta_p, result_delta_v;
    Eigen::Quaterniond result_delta_q;
    double dt = 0;
    if (first_update_) {
      integrator_->Integrate(dt, acc, gyro, acc, gyro, delta_p_, delta_q_,
                             delta_v_, linear_ba_, linear_bg_, &result_delta_p,
                             &result_delta_q, &result_delta_v, &linear_ba_,
                             &linear_bg_, true);
      last_acc_ = acc;
      last_gyro_ = gyro;
      last_timestamp_ = timestamp;
      first_update_ = false;
    } else {
      dt = timestamp - last_timestamp_;
      integrator_->Integrate(dt, last_acc_, last_gyro_, acc, gyro, delta_p_,
                             delta_q_, delta_v_, linear_ba_, linear_bg_,
                             &result_delta_p, &result_delta_q, &result_delta_v,
                             &linear_ba_, &linear_bg_, true);
      last_acc_ = acc;
      last_gyro_ = gyro;
      last_timestamp_ = timestamp;
      delta_p_ = result_delta_p;
      delta_q_ = result_delta_q;
      delta_v_ = result_delta_v;
    }
    Eigen::Quaterniond q_tmp;
    q_tmp = MathUtil::rpy_2_qua(MathUtil::qua_2_rpy(delta_q_));
    LOG(INFO) << "dt is: " << dt
              << " pose is: " << MathUtil::quat_2_axis(delta_q_).transpose()
              << " with gyro is: " << gyro.transpose()
              << " acc is: " << acc.transpose();
    geometry_msgs::msg::PoseStamped pose_rviz;
    pose_rviz.header.frame_id = "imu_odom";
    geometry_msgs::msg::Point position;
    position.x = 0;
    position.y = 0;
    position.z = 0;
    pose_rviz.pose.position = position;
    pose_rviz.pose.orientation.x = q_tmp.x();
    pose_rviz.pose.orientation.y = q_tmp.y();
    pose_rviz.pose.orientation.z = q_tmp.z();
    pose_rviz.pose.orientation.w = q_tmp.w();
    // pose_rviz.pose.orientation = odom->pose.pose.orientation;
    publisher_->publish(pose_rviz);
  }

  void OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr odom) {
    geometry_msgs::msg::PoseStamped pose_odom;
    pose_odom.header.frame_id = "odom";
    pose_odom.pose.position = odom->pose.pose.position;
    pose_odom.pose.orientation = odom->pose.pose.orientation;
    // publisher_->publish(pose_odom);
  }
  bool only_one_ = true;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  bool first_update_;
  double last_timestamp_;
  Eigen::Vector3d last_acc_;
  Eigen::Vector3d last_gyro_;
  ImuIntegrationInterfacePtr integrator_;
  Eigen::Vector3d delta_p_;
  Eigen::Vector3d delta_v_;
  Eigen::Quaterniond delta_q_;
  Eigen::Vector3d linear_ba_;
  Eigen::Vector3d linear_bg_;
  Eigen::Matrix3d transform_;
};
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<cartographer::mapping::ImuIntegrationTest> eskf_ptr =
      std::make_shared<cartographer::mapping::ImuIntegrationTest>();
  rclcpp::spin(eskf_ptr);
  rclcpp::shutdown();
  return 0;
}