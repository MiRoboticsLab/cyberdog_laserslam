/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#pragma once
#include <iostream>
#include <iomanip>
#include <fstream>

#include <thread>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "imu_integration/integration_helper/imu_odom_measurement_assembly.h"
#include "imu_integration/imu_odometry_eskf.h"
namespace imu_integration {
class ImuOdomEskfNode : public rclcpp::Node {
 public:
  ImuOdomEskfNode()
      : Node("imu_odom_eskf"),
        start_(false),
        first_odom_(true),
        previous_imu_time_(0.0),
        previous_odom_time_(0.0),
        process_thread_(nullptr) {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mi0016548/camera/imu", 10,
        std::bind(&ImuOdomEskfNode::ImuCallback, this, std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mi0016548/odom_out", 10,
        std::bind(&ImuOdomEskfNode::OdomCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "imu_odom_rviz", 10);
  }
  virtual ~ImuOdomEskfNode() {}

  bool Initialization() {
    collector_.reset(new ImuOdomMeasurementAssembly());
    double odom_sigma_velocity = 0.1;
    this->declare_parameter("odom_sigma_velocity");
    this->get_parameter("odom_sigma_velocity", odom_sigma_velocity);
    double odom_sigma_angular = 0.1;
    this->declare_parameter("odom_sigma_angular");
    this->get_parameter("odom_sigma_angular", odom_sigma_angular);
    std::cout << "odom sigma angular is: " << odom_sigma_angular << std::endl;
    double position_sigma = 0.2;
    this->declare_parameter("position_sigma");
    this->get_parameter("position_sigma", position_sigma);
    double velocity_sigma = 0.2;
    this->declare_parameter("velocity_sigma");
    this->get_parameter("velocity_sigma", velocity_sigma);
    double theta_sigma = 0.01;
    this->declare_parameter("theta_sigma");
    this->get_parameter("theta_sigma", theta_sigma);
    double ba_sigma = 0.1;
    this->declare_parameter("ba_sigma");
    this->get_parameter("ba_sigma", ba_sigma);
    double bg_sigma = 0.1;
    this->declare_parameter("bg_sigma");
    this->get_parameter("bg_sigma", bg_sigma);
    double gravity_sigma = 0.1;
    this->declare_parameter("gravity_sigma");
    this->get_parameter("gravity_sigma", gravity_sigma);
    double vi_sigma = 0.001;
    this->declare_parameter("vi_sigma");
    this->get_parameter("vi_sigma", vi_sigma);
    double thetai_sigma = 0.001;
    this->declare_parameter("thetai_sigma");
    this->get_parameter("thetai_sigma", thetai_sigma);
    double ai_sigma = 0.001;
    this->declare_parameter("ai_sigma");
    this->get_parameter("ai_sigma", ai_sigma);
    double wi_sigma = 0.001;
    this->declare_parameter("wi_sigma");
    this->get_parameter("wi_sigma", wi_sigma);
    bool update_jocabian = true;
    this->declare_parameter("update_jocabian");
    this->get_parameter("update_jocabian", update_jocabian);
    bool use_acc = false;
    this->declare_parameter("use_acc");
    this->get_parameter("use_acc", use_acc);
    int integration_type = 0;
    this->declare_parameter("integration_type");
    this->get_parameter("integration_type", integration_type);
    V3D gravity;
    std::cout << "velocity sigma is: " << velocity_sigma << std::endl;
    gravity << 0.0, 0.0, -kGravity;
    filter_.reset(new ImuOdometryEskf(
        odom_sigma_velocity, odom_sigma_angular, position_sigma, velocity_sigma,
        theta_sigma, ba_sigma, bg_sigma, gravity_sigma, vi_sigma, thetai_sigma,
        ai_sigma, wi_sigma, update_jocabian, use_acc, integration_type,
        gravity));
    filter_->Initialization();
    transform_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    std::cout << "Initialize successed" << std::endl;
    return true;
  }

  bool Start() {
    start_ = true;
    if (not process_thread_) {
      process_thread_.reset(
          new std::thread(std::bind(&ImuOdomEskfNode::Process, this)));
    } else {
      std::cerr << "imu odom thread exist already" << std::endl;
    }
    return true;
  }

  bool Stop() {
    start_ = false;
    if (process_thread_ && process_thread_->joinable()) {
      process_thread_->join();
    }
    return true;
  }

 private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    if (not start_) return;
    ImuMeasurement imu_data;
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
        imu->linear_acceleration.z;
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
        imu->angular_velocity.z;
    imu_data.acc = acc;
    imu_data.gyro = gyro;
    imu_data.timestamp =
        imu->header.stamp.sec + imu->header.stamp.nanosec * kNanosecToSec;

    collector_->CollectImuMeasurement(imu_data);
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    if (not start_) return;
    if (first_odom_) {
      last_odom_ = *odom;
      first_odom_ = false;
    } else {
      OdomMeasurement odom_meas;
      Eigen::Vector3d linear_velocity, angular_velocity;
      double timestamp =
          odom->header.stamp.sec + odom->header.stamp.nanosec * kNanosecToSec;
      double last_timestamp = last_odom_.header.stamp.sec +
                              last_odom_.header.stamp.nanosec * kNanosecToSec;
      double dt = timestamp - last_timestamp;
      Eigen::Vector3d delta_position;
      delta_position << odom->pose.pose.position.x -
                            last_odom_.pose.pose.position.x,
          odom->pose.pose.position.y - last_odom_.pose.pose.position.y,
          odom->pose.pose.position.z - last_odom_.pose.pose.position.z;
      double delta = delta_position.norm();
      double x_vel = delta / dt;
      linear_velocity << x_vel, 0.0, 0.0;
      // linear_velocity << odom->twist.twist.linear.x,
      // odom->twist.twist.linear.y,
      //     odom->twist.twist.linear.z;
      Eigen::Vector3d delta_angular;
      Eigen::Quaterniond last_pose(last_odom_.pose.pose.orientation.w,
                                   last_odom_.pose.pose.orientation.x,
                                   last_odom_.pose.pose.orientation.y,
                                   last_odom_.pose.pose.orientation.z);
      Eigen::Quaterniond pose(
          odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
          odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
      delta_angular =
          MathUtil::qua_2_rpy(pose) - MathUtil::qua_2_rpy(last_pose);
      // std::cout << "delta angular is: " << delta_angular.transpose()
      //           << std::endl;
      angular_velocity = delta_angular;
      // std::cout << "pose is " << MathUtil::qua_2_rpy(pose).transpose()
      //           << "last pose is : "
      //           << MathUtil::qua_2_rpy(last_pose).transpose() << std::endl;
      // std::fstream output;
      // output.open("/home/zfx/imu_integration.txt", std::ios::app);
      // output << std::setiosflags(std::ios::fixed) << std::setprecision(11)
      //        << angle(0) << "  " << angle(1) << "  " << angle(2) <<
      //        std::endl;
      // output.close();
      last_odom_ = *odom;
      std::cout << "odom angular is: " << angular_velocity.transpose()
                << std::endl;
      odom_meas.linear_velocity = linear_velocity;
      odom_meas.angular_velocity = angular_velocity;
      odom_meas.timestamp = timestamp;
      ImuOdomAssemblyMeasurement imu_odom_measurement;
      collector_->GetImuOdomAssemblyData(odom_meas, &imu_odom_measurement);
      if (not imu_odom_measurement.imu_measurements.empty()) {
        std::lock_guard<std::mutex> lk(imu_odom_lk_);
        imu_odom_meas_.push_back(imu_odom_measurement);
      }
    }
  }

  void Process() {
    if (not start_) return;
    while (start_) {
      std::lock_guard<std::mutex> lk(imu_odom_lk_);
      if (not imu_odom_meas_.empty()) {
        for (unsigned int i = 0;
             i < imu_odom_meas_.front().imu_measurements.size(); ++i) {
          ImuMeasurement imu;
          imu.acc =
              (imu_odom_meas_.front().imu_measurements[i].acc.transpose() *
               transform_)
                  .transpose();
          imu.gyro =
              (imu_odom_meas_.front().imu_measurements[i].gyro.transpose() *
               transform_)
                  .transpose();
          imu.timestamp = imu_odom_meas_.front().imu_measurements[i].timestamp;
          previous_imu_time_ = imu.timestamp;
          if (previous_imu_time_ > imu.timestamp) continue;
          filter_->PredictWithImu(imu.timestamp, imu.acc, imu.gyro);
        }
        double dt = (imu_odom_meas_.front().odom_measurement.timestamp -
                     previous_odom_time_);
        previous_odom_time_ = imu_odom_meas_.front().odom_measurement.timestamp;
        if (dt > 0.0) {
          filter_->UpdateWithOdom(
              imu_odom_meas_.front().odom_measurement.timestamp,
              imu_odom_meas_.front().odom_measurement.linear_velocity,
              imu_odom_meas_.front().odom_measurement.angular_velocity);
        }
        std::cout << "rotation is : "
                  << MathUtil::qua_2_rpy(filter_->state().rotation_).transpose()
                  << std::endl;
        std::cout << "position is: " << filter_->state().position_.transpose()
                  << std::endl;

        imu_odom_meas_.pop_front();
        geometry_msgs::msg::PoseStamped pose_rviz;
        pose_rviz.header.frame_id = "imu_odom";
        geometry_msgs::msg::Point position;
        position.x = filter_->state().position_.x();
        position.y = filter_->state().position_.y();
        position.z = filter_->state().position_.z();
        pose_rviz.pose.position = position;
        pose_rviz.pose.orientation.x = filter_->state().rotation_.x();
        pose_rviz.pose.orientation.y = filter_->state().rotation_.y();
        pose_rviz.pose.orientation.z = filter_->state().rotation_.z();
        pose_rviz.pose.orientation.w = filter_->state().rotation_.w();
        // pose_rviz.pose.orientation = odom->pose.pose.orientation;
        publisher_->publish(pose_rviz);
      }
    }
  }
  std::mutex imu_odom_lk_;
  bool start_;
  bool first_odom_;
  double previous_imu_time_;
  double previous_odom_time_;
  std::deque<ImuOdomAssemblyMeasurement> imu_odom_meas_;
  ImuOdomMeasurementAssemblyPtr collector_;
  ImuOdometryEskfPtr filter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::shared_ptr<std::thread> process_thread_;
  Eigen::Matrix3d transform_;
  nav_msgs::msg::Odometry last_odom_;
};
}  // namespace imu_integration