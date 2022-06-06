/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <deque>

#include "range_data_matching/filter/filter.h"
#include "transform/rigid_transform.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
constexpr double kSecToNanoSec = 1e9;
namespace cartographer {
namespace mapping {
class FilterTestNode : public rclcpp::Node {
 public:
  FilterTestNode() : Node("filter"), initial_(false), filter_(nullptr) {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mi1045856/camera/imu", 10,
        std::bind(&FilterTestNode::ImuCallback, this, std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mi1045856/odom_out", 10,
        std::bind(&FilterTestNode::OdomCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "imu_odom_rviz", 10);
  }
  virtual ~FilterTestNode() {}

  bool Initialization() {
    FilterParam param;
    param.position_sigma = 0.3;
    param.rotation_sigma = 0.01;
    param.velocity_sigma = 0.3;
    param.acc_bias_sigma = 0.1;
    param.gyro_bias_sigma = 0.01;
    param.vi_sigma = 0.01;
    param.wi_sigma = 0.001;
    param.thetai_sigma = 0.001;
    param.ai_sigma = 0.01;
    param.update_jocabian = true;
    param.odom_position_sigma = 0.01;
    param.odom_angular_sigma = 0.1;
    param.measure_ba_sigma = 0.0001;
    param.measure_bg_sigma = 0.0001;
    param.complementary_filter_alpha = 0.01;
    param.odom_2_imu_extrinsic = Eigen::Vector3d(-0.3252, -0.0475, 0.0795);
    filter_.reset(new Filter(param));
    transform_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    last_odom_.SetZero();
    initial_ = true;
    return true;
  }

  bool Start() {
    if (not process_thread_) {
      process_thread_.reset(
          new std::thread(std::bind(&FilterTestNode::Process, this)));
    } else {
      LOG(ERROR) << "imu odom thread exist already" << std::endl;
    }
    return true;
  }

  bool Stop() {
    initial_ = false;
    if (process_thread_ && process_thread_->joinable()) {
      process_thread_->join();
    }
    return true;
  }

 private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    ImuMeasurement imu_data;
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
        imu->linear_acceleration.z;
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
        imu->angular_velocity.z;
    imu_data.acc = (acc.transpose() * transform_).transpose();
    imu_data.gyro = (gyro.transpose() * transform_).transpose();
    imu_data.timestamp =
        imu->header.stamp.sec * kSecToNanoSec + imu->header.stamp.nanosec;
    LOG(INFO) << "imu timestamp : " << imu_data.timestamp
              << "acc is: " << imu_data.acc.transpose();

    ImuMeasurement imu_first;
    imu_first = imu_data;
    transform::Rigid3d pose =
        transform::Rigid3d::Rotation(Eigen::Quaterniond::FromTwoVectors(
            imu_first.acc, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d acc_new = (imu_first.acc.transpose() *
                               pose.rotation().toRotationMatrix().inverse())
                                  .transpose();
    LOG(INFO) << "correct imu: " << acc_new.transpose()
              << "with imu: " << imu_first.acc.transpose();
    {
      std::lock_guard<std::mutex> lk(imu_lk_);
      imu_data_.push_back(imu_data);
    }
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    if (last_odom_.IsZero()) {
      Eigen::Vector3d position, angular;
      position << odom->pose.pose.position.x, odom->pose.pose.position.y,
          odom->pose.pose.position.z;
      angular = MathUtil::quat_2_axis(
          Q4D(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z));
      double timestamp =
          odom->header.stamp.sec * kSecToNanoSec + odom->header.stamp.nanosec;
      OdomMeasurement mea;
      mea.position = position;
      mea.angular = angular;
      mea.timestamp = timestamp;
      last_odom_ = mea;
    } else {
      OdomMeasurement odom_meas;
      Eigen::Vector3d linear_velocity, angular_velocity, position, angular;
      double timestamp =
          odom->header.stamp.sec * kSecToNanoSec + odom->header.stamp.nanosec;
      position << odom->pose.pose.position.x, odom->pose.pose.position.y,
          odom->pose.pose.position.z;
      angular = MathUtil::quat_2_axis(
          Q4D(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z));
      LOG(INFO) << "odom pose is: " << angular.transpose()
                << "odom position is: " << position.transpose();
      odom_meas.timestamp = timestamp;
      odom_meas.position = position;
      odom_meas.angular = angular;
      double dt = (timestamp - last_odom_.timestamp) * kNanoSecToSec;
      Eigen::Vector3d delta_position;
      delta_position << odom->pose.pose.position.x - last_odom_.position.x(),
          odom->pose.pose.position.y - last_odom_.position.y(),
          odom->pose.pose.position.z - last_odom_.position.z();
      linear_velocity = delta_position / dt;
      Eigen::Vector3d delta_angular;

      Eigen::Quaterniond pose(
          odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
          odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
      delta_angular = MathUtil::quat_2_axis(pose) - last_odom_.angular;
      angular_velocity = delta_angular / dt;
      last_odom_ = odom_meas;
      OdomMeasurement velocity_meas;
      velocity_meas.timestamp = timestamp;
      velocity_meas.position = pose.inverse() * linear_velocity;
      LOG(INFO) << "odom velocity is: " << velocity_meas.position.transpose();
      velocity_meas.angular = angular_velocity;
      {
        std::lock_guard<std::mutex> lk(odom_lk_);
        odom_data_.push_back(velocity_meas);
      }
    }
  }

  void Process() {
    while (initial_) {
      usleep(1e5);
      LOG(INFO) << "size of data is: " << imu_data_.size()
                << "odom size is: " << odom_data_.size();
      if (!imu_data_.empty() && !odom_data_.empty()) {
        LOG(INFO) << "1";
        double time = odom_data_.back().timestamp < imu_data_.back().timestamp
                          ? odom_data_.back().timestamp
                          : imu_data_.back().timestamp;
        std::vector<ImuMeasurement> imu_process;
        std::vector<OdomMeasurement> odom_process;
        {
          std::lock_guard<std::mutex> lk_imu(imu_lk_);
          std::lock_guard<std::mutex> lk_odom(odom_lk_);
          while (imu_data_.front().timestamp < time) {
            LOG(INFO) << "imu timestamp : " << imu_data_.front().timestamp;
            imu_process.push_back(imu_data_.front());
            imu_data_.pop_front();
          }
          while (odom_data_.front().timestamp < time) {
            LOG(INFO) << "odom timestamp : " << odom_data_.front().timestamp
                      << "linear vel: "
                      << odom_data_.front().position.transpose();
            odom_process.push_back(odom_data_.front());
            odom_data_.pop_front();
          }
        }
        LOG(INFO) << "3";
        filter_->Advance(time, imu_process, odom_process);
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

  bool initial_;
  std::mutex imu_lk_;
  std::mutex odom_lk_;
  std::deque<ImuMeasurement> imu_data_;
  std::deque<OdomMeasurement> odom_data_;
  Eigen::Matrix3d transform_;
  OdomMeasurement last_odom_;
  FilterPtr filter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::shared_ptr<std::thread> process_thread_;
};
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<cartographer::mapping::FilterTestNode> filter =
      std::make_shared<cartographer::mapping::FilterTestNode>();
  filter->Initialization();
  filter->Start();
  rclcpp::spin(filter);
  rclcpp::shutdown();
  filter->Stop();
  return 0;
}
