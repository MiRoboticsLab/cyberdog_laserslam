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

#include "pose_extrapolator/filter/tracker.h"
#include "pose_extrapolator/filter/imu_tracker.h"
#include "pose_extrapolator/pose_extrapolator_interface.h"
#include "pose_extrapolator/pose_extrapolator.h"
#include "pose_extrapolator/measurement_collector.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
constexpr double kSecToNanoSec = 1e9;
namespace cartographer {
namespace pose_extrapolator {
class TrackerTestNode : public rclcpp::Node {
 public:
  TrackerTestNode()
      : Node("tracker"),
        start_(false),
        tracker_(nullptr),
        imu_tracker_(nullptr),
        pose_extrapolator_(nullptr),
        collector_(nullptr) {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mi1045856/camera/imu", 10,
        std::bind(&TrackerTestNode::ImuCallBack, this, std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mi1045856/odom_out", 10,
        std::bind(&TrackerTestNode::OdomCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "imu_odom_rviz", 10);
    transform_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    cartographer::FilterParam param;
    param.integration_type = 0;
    param.alpha = 0.01;
    param.use_filter = false;
    param.gravity_constant = 10.0;
    param.pose_duration_time = 0.001;
    param_ = param;
    tracker_.reset(new filter::Tracker(param));
    collector_.reset(new MeasurementCollector(param));
  }
  virtual ~TrackerTestNode() {}

  bool Start() {
    start_ = true;
    if (not process_thread_) {
      process_thread_.reset(
          new std::thread(std::bind(&TrackerTestNode::Process, this)));
    } else {
      LOG(ERROR) << "imu odom thread exist already" << std::endl;
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
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu) {
    filter::Tracker::ImuMeasurement imu_meas;
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
    collector_->AddImuData(imu_meas);
    {
      std::lock_guard<std::mutex> lk(time_lk_);
      newest_time_ = imu_meas.time;
    }
    // if (pose_extrapolator_ == nullptr) {
    //   std::vector<sensor::ImuData> meas;
    //   meas.push_back(imu_meas);
    //   std::vector<transform::TimedRigid3d> poses;
    //   poses.push_back(transform::TimedRigid3d{transform::Rigid3d::Identity(),
    //                                           imu_meas.time});
    //   LOG(INFO) << "time of imu is: " << imu_meas.time;
    //   pose_extrapolator_ =
    //       pose_extrapolator::PoseExtraPolatorInterface::CreateWithImuData(
    //           param_, meas, poses);
    //   LOG(INFO) << "pose extrapolator initialized";
    //   return;
    // }
    // {
    //   std::lock_guard<std::mutex> lk(imu_lk_);
    //   pose_extrapolator_->AddImuData(imu_meas);
    // }
    // if (imu_tracker_) {
    //   std::lock_guard<std::mutex> lk(imu_lk_);
    //   imu_tracker_->AddImuLinearAccelerationObservation(
    //       imu_meas.linear_acceleration);
    //   imu_tracker_->AddImuAngularVelocityObservation(imu_meas.angular_velocity);
    // }
    // LOG(INFO) << "imu time is: " << imu_meas.time;
    // tracker_->AddImuData(imu_meas);
  }
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    Eigen::Vector3d position;
    Eigen::Quaterniond angular;
    position << odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0;
    angular = Q4D(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    filter::Tracker::OdomMeasurement odom_meas;
    transform::Rigid3d pose(position, angular);
    odom_meas.pose = pose;
    odom_meas.time =
        common::FromUniversal((odom->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (odom->header.stamp.nanosec + 50) / 100);
    collector_->AddOdometryData(odom_meas);
    // if (pose_extrapolator_ == nullptr) {
    //   // still not initialized
    //   LOG(INFO) << "return";
    //   return;
    // }
    // if (odom_meas.time < pose_extrapolator_->GetLastPoseTime()) {
    //   return;
    // }
    // LOG(INFO) << "add odom";
    // {
    //   std::lock_guard<std::mutex> lk(odom_lk_);
    //   pose_extrapolator_->AddOdometryData(odom_meas);
    // }
    // LOG(INFO) << "add odom end";

    // if (not imu_tracker_) {
    //   LOG(INFO) << "reset imu tracker";
    //   imu_tracker_.reset(new filter::ImuTracker(10, odom_meas.time));
    // }
    // LOG(INFO) << "odom time is: " << odom_meas.time;
    // tracker_->AddOdometryData(odom_meas);
  }

  void Process() {
    // tracker_->AddPose(transform::TimedRigid3d{transform::Rigid3d::Identity(),
    //                                           common::FromUniversal(0)});
    while (start_) {
      usleep(2e5);
      LOG(INFO) << "process";
      common::Time time;
      {
        std::lock_guard<std::mutex> lk(time_lk_);
        time = newest_time_;
      }
      // LOG(INFO) << "time is : " << time;
      LOG(INFO) << "newest timne is: " << time;
      //   transform::Rigid3d pose = tracker_->Advance(time);
      //   tracker_->AddPose(transform::TimedRigid3d{pose, time});
      //   LOG(INFO) << "lets go";
      //   Eigen::Quaterniond orientation;
      //   if (imu_tracker_) {
      //     std::lock_guard<std::mutex> lk(imu_lk_);
      //     imu_tracker_->Advance(time);
      //     orientation = imu_tracker_->orientation();
      //   }
      transform::Rigid3d pose;
      pose = collector_->ExtrapolatePose(time);
      collector_->AddPose(transform::TimedRigid3d{pose, time});
      // LOG(INFO) << "djfk";
      // if (pose_extrapolator_ != nullptr) {
      //   std::lock_guard<std::mutex> lk(imu_lk_);
      //   std::lock_guard<std::mutex> lk_odom(odom_lk_);
      //   LOG(INFO) << "1";
      //   common::Time newest_time;
      //   newest_time = newest_time_;
      //   LOG(INFO) << "time is: " << newest_time;
      //   if (newest_time == common::Time::min()) {
      //     LOG(INFO) << "do nothing until odom come";
      //   } else {
      //     pose = pose_extrapolator_->ExtrapolatePose(newest_time);
      //     LOG(INFO) << "2";
      //     pose_extrapolator_->AddPose(
      //         transform::TimedRigid3d{pose, newest_time});
      //     LOG(INFO) << "add pose";
      //   }
      // }
      geometry_msgs::msg::PoseStamped pose_rviz;
      pose_rviz.header.frame_id = "imu_odom";
      geometry_msgs::msg::Point position;
      position.x = pose.translation().x();
      position.y = pose.translation().y();
      position.z = pose.translation().z();
      pose_rviz.pose.position = position;
      pose_rviz.pose.orientation.x = pose.rotation().x();
      pose_rviz.pose.orientation.y = pose.rotation().y();
      pose_rviz.pose.orientation.z = pose.rotation().z();
      pose_rviz.pose.orientation.w = pose.rotation().w();
      publisher_->publish(pose_rviz);
      LOG(INFO) << "imu tracker pose is: "
                << transform::RotationQuaternionToAngleAxisVector(
                       pose.rotation())
                       .transpose();
    }
  }
  bool start_;
  std::mutex time_lk_;
  std::mutex imu_lk_;
  std::mutex odom_lk_;
  FilterParam param_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  filter::TrackerPtr tracker_;
  Eigen::Matrix3d transform_;
  std::shared_ptr<std::thread> process_thread_;
  common::Time newest_time_;
  std::shared_ptr<filter::ImuTracker> imu_tracker_;
  std::unique_ptr<PoseExtraPolatorInterface> pose_extrapolator_;
  std::unique_ptr<MeasurementCollector> collector_;
};
}  // namespace pose_extrapolator
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  std::shared_ptr<cartographer::pose_extrapolator::TrackerTestNode> tracker =
      std::make_shared<cartographer::pose_extrapolator::TrackerTestNode>();
  tracker->Start();
  rclcpp::spin(tracker);
  tracker->Stop();
  rclcpp::shutdown();
  return 0;
}
