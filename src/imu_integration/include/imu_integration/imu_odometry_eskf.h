/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_IMU_ODOMETRY_ESKF_H_
#define IMU_INTEGRATION_IMU_ODOMETRY_ESKF_H_
#include "imu_integration/error_state_kalman_filter.h"
constexpr double kNanosecToSec = 1e-9;
constexpr double kSecToNanoSec = 1e9;
constexpr double kPulishNum = 0.005;
constexpr double kPulishThreshold = 0.005;
namespace imu_integration {
class ImuOdometryEskf {
 public:
  ImuOdometryEskf(double odom_sigma_velocity, double odom_sigma_angular,
                  double position_sigma, double velocity_sigma,
                  double theta_sigma, double ba_sigma, double bg_sigma,
                  double gravity_sigma, double vi_sigma, double thetai_sigma,
                  double ai_sigma, double wi_sigma, bool update_jocabian,
                  bool use_acc, int integration_type, const V3D& gravity)
      : odom_sigma_velocity_(odom_sigma_velocity),
        odom_sigma_angular_(odom_sigma_angular),
        first_imu_time_(0.0),
        last_imu_time_(0.0),
        last_odom_time_(0.0),
        is_first_(true),
        first_update_(true) {
    last_acc_.setZero();
    last_gyro_.setZero();
    filter_ = std::make_shared<ErrorStateKalmanFilter>(
        position_sigma, velocity_sigma, theta_sigma, ba_sigma, bg_sigma,
        gravity_sigma, vi_sigma, thetai_sigma, ai_sigma, wi_sigma,
        update_jocabian, use_acc, integration_type, gravity);
  }
  virtual ~ImuOdometryEskf() {}

  bool Initialization();

  GlobalState state() const { return filter_->state(); }

  void PredictWithImu(double timestamp, const Eigen::Vector3d& acc,
                      const Eigen::Vector3d& gyro);

  void UpdateWithOdom(double timestamp, const Eigen::Vector3d& linear_velocity,
                      const Eigen::Vector3d& angular_velocity);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void UpdateState(double dt, const Eigen::Vector3d& linear_velocity,
                   const Eigen::Vector3d& angular_velocity,
                   const Eigen::Matrix<double, 15, 6>& kalman_gain,
                   const Eigen::Matrix<double, 15, 15>& covariance);
  double odom_sigma_velocity_;
  double odom_sigma_angular_;
  double first_imu_time_;
  double last_imu_time_;
  double last_odom_time_;
  bool is_first_;
  bool first_update_;
  Eigen::Vector3d last_acc_;
  Eigen::Vector3d last_gyro_;
  GlobalState state_;
  ErrorStateKalmanFilterPtr filter_;
};
typedef std::shared_ptr<ImuOdometryEskf> ImuOdometryEskfPtr;
typedef std::shared_ptr<const ImuOdometryEskf> ImuOdometryEskfConstPtr;
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_IMU_ODOMETRY_ESKF_H_
