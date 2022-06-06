/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_FILTER_FILTER_H_
#define RANGE_DATA_MATCHING_FILTER_FILTER_H_
#include <memory>
#include <vector>
#include <thread>
#include <mutex>

#include <glog/logging.h>

#include "range_data_matching/filter/filter_helper/global_state.h"
#include "range_data_matching/filter/imu_integration_interface.h"
#include "range_data_matching/filter/imu_integration_midpoint.h"
#include "range_data_matching/filter/filter_helper/imu_odom_measurement_assembly.h"
#include "range_data_matching/filter/complementary_filter.h"

constexpr double kGravityConstant = 9.81;
constexpr int kMeasurementDim = 9;
constexpr double kNanoSecToSec = 1e-9;
namespace cartographer {
namespace mapping {
struct FilterParam {
  double position_sigma;
  double rotation_sigma;
  double velocity_sigma;
  double acc_bias_sigma;
  double gyro_bias_sigma;
  double vi_sigma;
  double wi_sigma;
  double thetai_sigma;
  double ai_sigma;
  bool update_jocabian;
  double odom_position_sigma;
  double odom_angular_sigma;
  double measure_ba_sigma;
  double measure_bg_sigma;
  double complementary_filter_alpha;
  V3D odom_2_imu_extrinsic;
};
class Filter {
 public:
  explicit Filter(const FilterParam& param)
      : update_jocabian_(param.update_jocabian),
        first_update_(true),
        last_odom_update_time_(0.0),
        odom_position_sigma_(param.odom_position_sigma),
        odom_angular_sigma_(param.odom_angular_sigma),
        measure_ba_sigma_(param.measure_ba_sigma),
        measure_bg_sigma_(param.measure_bg_sigma),
        newest_updated_time_(0.0),
        odom_2_imu_extrinsic_(param.odom_2_imu_extrinsic),
        complementary_filter_(nullptr) {
    MXD covariance;
    covariance.resize(kStateDim, kStateDim);
    covariance.setZero();
    covariance.block<3, 3>(kPosition, kPosition) =
        V3D(param.position_sigma, param.position_sigma,
            param.position_sigma)
            .asDiagonal();  // position
    covariance.block<3, 3>(kVelocity, kVelocity) =
        V3D(param.velocity_sigma, param.velocity_sigma,
            param.velocity_sigma)
            .asDiagonal();  // velocity
    covariance.block<3, 3>(kRotation, kRotation) =
        V3D(param.rotation_sigma, param.rotation_sigma,
            param.rotation_sigma)
            .asDiagonal();  // theta
    covariance.block<3, 3>(kAccBias, kAccBias) =
        V3D(param.acc_bias_sigma, param.acc_bias_sigma, param.acc_bias_sigma)
            .asDiagonal();  // ba
    covariance.block<3, 3>(kGyroBias, kGyroBias) =
        V3D(param.gyro_bias_sigma, param.gyro_bias_sigma, param.gyro_bias_sigma)
            .asDiagonal();  // bg
    MXD noise;
    noise.resize(kNoiseDim, kNoiseDim);
    noise.setZero();
    noise.block<3, 3>(0, 0) =
        V3D(param.vi_sigma, param.vi_sigma, param.vi_sigma).asDiagonal();
    noise.block<3, 3>(3, 3) =
        V3D(param.wi_sigma, param.wi_sigma, param.wi_sigma).asDiagonal();
    noise.block<3, 3>(6, 6) =
        V3D(param.thetai_sigma, param.thetai_sigma, param.thetai_sigma)
            .asDiagonal();
    noise.block<3, 3>(9, 9) =
        V3D(param.ai_sigma, param.ai_sigma, param.ai_sigma).asDiagonal();

    V3D gravity(0.0, 0.0, -kGravityConstant);
    integrator_.reset(new ImuIntegrationMidPoint(covariance, noise, gravity));
    last_imu_data_.acc.setZero();
    last_imu_data_.gyro.setZero();
    last_imu_data_.timestamp = 0.0;
    ba_measurement_ << 0.0, 0.0, 0.0;
    bg_measurement_ << 0.0, 0.0, 0.0;
    newest_linear_velocity_ << 0.0, 0.0, 0.0;
    newest_angular_velocity_ << 0.0, 0.0, 0.0;
    state_.SetIdentity();
    covariance_ = integrator_->covariance();
    complementary_filter_.reset(
        new ComplementaryFilter(param.complementary_filter_alpha));
  }
  virtual ~Filter() {}

  void Advance(double time, const std::vector<ImuMeasurement>& imu_data,
               const std::vector<OdomMeasurement>& odom_data);

  const GlobalState& state() const { return state_; }

  double newest_updated_time() const { return newest_updated_time_; }

  const V3D& newest_linear_velocity() const { return newest_linear_velocity_; }

  const V3D& newest_angular_velocity() const {
    return newest_angular_velocity_;
  }

 private:
  void Predict(const std::vector<ImuMeasurement>& imu_datas);

  void UpdateWithOdom(double dt, const V3D& velocity, const V3D& position);

  std::vector<ImuOdomAssemblyMeasurement> FindAssemblyMeasurement(
      double time, const std::vector<ImuMeasurement>& imu_data,
      const std::vector<OdomMeasurement>& odom_data);

  void UpdatePositionToNewest(double dt);

  void UpdateNewestLinearVelocity(const V3D& velocity) {
    newest_linear_velocity_ = velocity;
  }

  void UpdateNewestAngularVelocity(double dt, const Q4D& last_pose,
                                   const Q4D& pose_now) {
    V3D delta_axis =
        MathUtil::quat_2_axis(pose_now) - MathUtil::quat_2_axis(last_pose);
    newest_angular_velocity_ = delta_axis / dt;
  }
  // According to Paper "A Mathematical Introduction to Robotic Manipulation"
  // Velocity Transform between two coordinate within same rigid object
  void TransformOdomVelocityToImuFrame(const V3D& odom_velocity, double omega,
                                       V3D* imu_velocity);

  bool update_jocabian_;
  bool first_update_;
  double last_odom_update_time_;
  double odom_position_sigma_;
  double odom_angular_sigma_;
  double measure_ba_sigma_;
  double measure_bg_sigma_;
  double newest_updated_time_;
  V3D odom_2_imu_extrinsic_;
  GlobalState state_;
  GlobalState last_updated_state_;
  MXD covariance_;
  ImuIntegrationInterfacePtr integrator_;
  ImuMeasurement last_imu_data_;
  V3D ba_measurement_;
  V3D bg_measurement_;
  V3D newest_linear_velocity_;
  V3D newest_angular_velocity_;
  ComplementaryFilterPtr complementary_filter_;
};
typedef std::shared_ptr<const Filter> FilterConstPtr;
typedef std::shared_ptr<Filter> FilterPtr;
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_FILTER_FILTER_H_
