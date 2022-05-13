/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_extrapolator/filter/tracker.h"

namespace cartographer {
namespace pose_extrapolator {
namespace filter {
Tracker::Tracker(const FilterParam& param)
    : first_advance_(true), first_imu_(true) {
  if (param.integration_type == 0) {
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
    complementary_filter_.reset(new filter::ComplementaryFilter(param.alpha));
    integrator_.reset(
        new integrator::ImuIntegrationMidPoint(covariance, noise, gravity));
    newest_angular_velocity_from_imu_.setZero();
    newest_velocity_.setZero();
  }
}
// TO_DO(feixiang): change the last measurement problem
void Tracker::AddImuData(const ImuMeasurement& imu_data) {
  {
    std::lock_guard<std::mutex> lk(imu_lk_);
    imu_datas_.push_back(imu_data);
    if (first_imu_) {
      pose_updated_ = transform::TimedRigid3d{transform::Rigid3d::Identity(),
                                              imu_data.time};
      first_imu_ = false;
    }
  }
}

void Tracker::AddOdometryData(const OdomMeasurement& odom_data) {
  {
    std::lock_guard<std::mutex> lk(odom_lk_);
    odom_datas_.push_back(odom_data);
    if (first_advance_) {
      last_odom_meas_ = odom_data;
      first_advance_ = false;
    }
  }
}

void Tracker::TrimImuData(const common::Time& time) {
  while (imu_datas_.front().time < time && not imu_datas_.empty()) {
    LOG(INFO) << "imu time " << imu_datas_.front().time << " time is: " << time;
    imu_datas_.pop_front();
    LOG(INFO) << "pop";
  }
  LOG(INFO) << "ok";
}

void Tracker::TrimOdometryData(const common::Time& time) {
  while (odom_datas_.front().time < time && not odom_datas_.empty()) {
    odom_datas_.pop_front();
  }
}

void Tracker::AddPose(const transform::TimedRigid3d& pose) {
  std::lock_guard<std::mutex> lk(pose_lk_);
  pose_updated_ = pose;
}

transform::Rigid3d Tracker::TrackPose(const common::Time& time,
                                      std::deque<ImuMeasurement>& imu_datas,
                                      std::deque<OdomMeasurement>& odom_datas) {
  if (imu_datas.empty() || odom_datas.empty()) {
    return transform::Rigid3d::Identity();
  }
  transform::TimedRigid3d pose_updated;
  {
    std::lock_guard<std::mutex> lk(pose_lk_);
    pose_updated = pose_updated_;
  }
  while (not imu_datas.empty() &&
         imu_datas.front().time < pose_updated.timestamp) {
    imu_datas.pop_front();
  }
  while (not odom_datas.empty() &&
         odom_datas.front().time < pose_updated.timestamp) {
    odom_datas.pop_front();
  }
  common::Time delta_pose_start_time = imu_datas.front().time;
  transform::TimedRigid3d delta_pose = transform::TimedRigid3d{
      transform::Rigid3d::Identity(), delta_pose_start_time};
  Eigen::Quaterniond rotation;
  std::vector<transform::TimedRigid3d> poses_from_imu;
  double delta_time =
      common::ToSeconds(delta_pose_start_time - pose_updated.timestamp);
  LOG(INFO) << "imu delta time is : " << delta_time
            << "imu time is: " << imu_datas.front().time
            << "newest angular velocity is: "
            << newest_angular_velocity_from_imu_.transpose()
            << "imu data size : " << imu_datas.size();
  Eigen::Vector3d delta_angular =
      newest_angular_velocity_from_imu_ * delta_time;
  Eigen::Quaterniond intepolated_rotation =
      transform::AngleAxisVectorToRotationQuaternion(delta_angular) *
      pose_updated.pose.rotation();
  Eigen::Vector3d intepolated_translation =
      pose_updated.pose.translation() + newest_velocity_ * delta_time;
  transform::TimedRigid3d intepolated_pose = transform::TimedRigid3d{
      transform::Rigid3d(intepolated_translation, intepolated_rotation),
      delta_pose_start_time};
  poses_from_imu.push_back(intepolated_pose);
  while (imu_datas.size() > 2 && imu_datas[1].time < time) {
    double dt = common::ToSeconds(imu_datas[1].time - imu_datas.front().time);
    integrator_->IntegrateAngleOnly(dt, imu_datas.front().angular_velocity,
                                    imu_datas[1].angular_velocity,
                                    delta_pose.pose.rotation(), &rotation);
    Eigen::Quaterniond filter_pose;
    complementary_filter_->Update(rotation, imu_datas[1].linear_acceleration,
                                  &filter_pose);

    rotation = filter_pose;
    delta_pose = transform::TimedRigid3d{transform::Rigid3d::Rotation(rotation),
                                         imu_datas[1].time};
    Eigen::Quaterniond orientation =
        (delta_pose.pose.rotation() * intepolated_pose.pose.rotation())
            .normalized();
    poses_from_imu.push_back(transform::TimedRigid3d{
        transform::Rigid3d::Rotation(orientation), imu_datas[1].time});
    imu_datas.pop_front();
  }
  if (imu_datas.size() == 2 && imu_datas[1].time < time) {
    double delta_last_time =
        common::ToSeconds(imu_datas[1].time - imu_datas.front().time);
    integrator_->IntegrateAngleOnly(
        delta_last_time, imu_datas.front().angular_velocity,
        imu_datas[1].angular_velocity, delta_pose.pose.rotation(), &rotation);
    Eigen::Quaterniond filter_pose;
    complementary_filter_->Update(rotation, imu_datas[1].linear_acceleration,
                                  &filter_pose);
    rotation = filter_pose;
    Eigen::Quaterniond orientation =
        (delta_pose.pose.rotation() * intepolated_pose.pose.rotation())
            .normalized();
    poses_from_imu.push_back(transform::TimedRigid3d{
        transform::Rigid3d::Rotation(orientation), imu_datas[1].time});
  }
  for (unsigned int i = 0; i < poses_from_imu.size(); ++i) {
    LOG(INFO) << transform::RotationQuaternionToAngleAxisVector(
                     poses_from_imu[i].pose.rotation())
                     .transpose();
  }
  // some nasty things such as nan occur when the dt is 0
  double dt_imu =
      common::ToSeconds(delta_pose.timestamp - delta_pose_start_time);
  if (dt_imu < 1e-10) {
    LOG(WARNING) << "dt too short";
    newest_angular_velocity_from_imu_ = imu_datas.back().angular_velocity;
  } else {
    newest_angular_velocity_from_imu_ =
        transform::RotationQuaternionToAngleAxisVector(
            delta_pose.pose.rotation()) /
        common::ToSeconds(delta_pose.timestamp - delta_pose_start_time);
  }
  // transform::TimedRigid3d result_pose = pose_updated;
  // while (odom_datas.size() > 1 && odom_datas.front().time < time) {
  //   Eigen::Vector3d linear_velocity_from_odometry =
  //       (odom_datas.front().pose.translation() -
  //        last_odom_meas_.pose.translation()) /
  //       common::ToSeconds(odom_datas.front().time - last_odom_meas_.time);
  //   transform::TimedRigid3d start_pose, end_pose;
  //   bool start = false, end = false;

  //   for (unsigned int i = 0; i < poses_from_imu.size(); ++i) {
  //     if (poses_from_imu[i].timestamp < odom_datas.front().time) {
  //       start_pose = poses_from_imu[i];
  //       start = true;
  //     }
  //     if (poses_from_imu[i].timestamp >= odom_datas.front().time) {
  //       end_pose = poses_from_imu[i];
  //       end = true;
  //       break;
  //     }
  //   }
  //   transform::TimedRigid3d odom_pose =
  //       transform::Interpolate(start_pose, end_pose,
  //       odom_datas.front().time);
  //   LOG(INFO) << "start pose is : " << start_pose.pose
  //             << "end pose is: " << end_pose.pose
  //             << "odom pose : " << odom_pose.pose;
  //   linear_velocity_from_odometry =
  //       odom_pose.pose.rotation() * linear_velocity_from_odometry;
  //   newest_velocity_ = linear_velocity_from_odometry;
  //   Eigen::Vector3d new_translation =
  //       result_pose.pose.translation() +
  //       linear_velocity_from_odometry *
  //           common::ToSeconds(odom_datas.front().time -
  //           result_pose.timestamp);
  //   LOG(INFO) << "new traslation is: " << new_translation.transpose()
  //             << "velocity is: " <<
  //             linear_velocity_from_odometry.transpose();
  //   LOG(INFO) << "pose updated translation is: "
  //             << result_pose.pose.translation().transpose();
  //   result_pose = transform::TimedRigid3d{
  //       transform::Rigid3d(new_translation, odom_pose.pose.rotation()),
  //       odom_pose.timestamp};
  //   last_odom_meas_ = odom_datas.front();
  //   odom_datas.pop_front();
  // }

  // double delta_time_from_timepoint_to_odom =
  //     common::ToSeconds(time - result_pose.timestamp);
  // LOG(INFO) << "angular velocity is: "
  //           << newest_angular_velocity_from_imu_.transpose()
  //           << "delta time is: " << delta_time_from_timepoint_to_odom;
  transform::Rigid3d result_q_t;
  // Eigen::Vector3d delta_q =
  //     newest_angular_velocity_from_imu_ * delta_time_from_timepoint_to_odom;
  // Eigen::Quaterniond result_r =
  //     transform::AngleAxisVectorToRotationQuaternion(delta_q) *
  //     result_pose.pose.rotation();
  // Eigen::Vector3d result_t =
  //     result_pose.pose.translation() +
  //     newest_velocity_ * delta_time_from_timepoint_to_odom;
  Eigen::Quaterniond result_r =
      delta_pose.pose.rotation() * pose_updated.pose.rotation();
  Eigen::Vector3d result_t = Eigen::Vector3d::Zero();
  result_q_t = transform::Rigid3d(result_t, result_r);
  return result_q_t;
  // transform::TimedRigid3d pose1 = pose_updated_;
  // std::vector<transform::TimedRigid3d> timed_pose_queue;
  // // TO_DO(feixiang): change the pose1 to the imu integrated pose, because
  // the
  // // orientation should be calculated by imu gyro other than the pose from
  // odom
  // // and intepolate
  // while (imu_datas.front().time < time) {
  //   LOG(INFO) << "last imu meas time is: " << last_imu_meas_.time;
  //   double dt = common::ToSeconds(imu_datas.front().time -
  //   last_imu_meas_.time); Eigen::Vector3d velocity, ba, bg,
  //       position;  // no use, just for suit interface
  //   Eigen::Quaterniond result_q;
  //   integrator_->Integrate(
  //       dt, last_imu_meas_.linear_acceleration,
  //       last_imu_meas_.angular_velocity,
  //       imu_datas.front().linear_acceleration,
  //       imu_datas.front().angular_velocity, pose1.pose.translation(),
  //       pose1.pose.rotation(), velocity, ba, bg, &position, &result_q,
  //       &velocity, &ba, &bg, false);
  //   Eigen::Vector3d pos;
  //   pos.setZero();
  //   transform::TimedRigid3d pose{transform::Rigid3d(pos, result_q),
  //                                imu_datas.front().time};
  //   pose1 = pose;
  //   last_imu_meas_ = imu_datas.front();
  //   imu_datas.pop_front();
  //   newest_angular_ =
  //       transform::RotationQuaternionToAngleAxisVector(
  //           (pose.pose.inverse() * timed_pose_queue.back().pose).rotation())
  //           /
  //       dt;
  //   timed_pose_queue.push_back(pose);
  // }
  // transform::TimedRigid3d pose = pose_updated_;
  // while (odom_datas.front().time < time) {
  //   if (odom_datas.front().time == last_odom_meas_.time) {
  //     continue;
  //   }
  //   transform::TimedRigid3d odom_pose;
  //   for (size_t i = 1; i < timed_pose_queue.size(); ++i) {
  //     if (odom_datas.front().time <= timed_pose_queue[i].timestamp &&
  //         odom_datas.front().time >= timed_pose_queue[i - 1].timestamp) {
  //       odom_pose =
  //           transform::Interpolate(timed_pose_queue[i - 1],
  //           timed_pose_queue[i],
  //                                  odom_datas.front().time);
  //       break;
  //     }
  //   }
  //   double delta_time = common::ToSeconds(odom_pose.timestamp -
  //   pose.timestamp); double delta_time_odom =
  //       common::ToSeconds(odom_datas.front().time - last_odom_meas_.time);
  //   Eigen::Vector3d velocity = (odom_datas.front().pose.translation() -
  //                               last_odom_meas_.pose.translation()) /
  //                              delta_time_odom;
  //   velocity = odom_pose.pose.rotation() * velocity;
  //   newest_velocity_ = velocity;
  //   Eigen::Vector3d delta_p = velocity * delta_time;
  //   Eigen::Vector3d translation = pose.pose.translation() + delta_p;
  //   pose = transform::TimedRigid3d{
  //       transform::Rigid3d(translation, odom_pose.pose.rotation()),
  //       odom_pose.timestamp};
  // }
  // const double delta_time = common::ToSeconds(time - pose.timestamp);
  // Eigen::Vector3d delta_translation = newest_velocity_ * delta_time;
  // const Eigen::Vector3d translation_new =
  //     pose.pose.translation() + delta_translation;
  // const Eigen::Quaterniond rotation_new =
  //     (pose.pose.rotation() *
  //      transform::AngleAxisVectorToRotationQuaternion<double>(newest_angular_
  //      *
  //                                                             delta_time))
  //         .normalized();
  // return transform::Rigid3d(translation_new, rotation_new);
}

transform::Rigid3d Tracker::Advance(const common::Time& time) {
  if (imu_datas_.empty() || odom_datas_.empty()) {
    return transform::Rigid3d::Identity();
  }
  LOG(INFO) << "begin process"
            << " imu size is: " << imu_datas_.size()
            << "odom size is: " << odom_datas_.size();
  std::deque<ImuMeasurement> imu_meas;
  std::deque<OdomMeasurement> odom_meas;
  {
    std::lock_guard<std::mutex> lk(imu_lk_);
    imu_meas = imu_datas_;
    TrimImuData(time);
  }
  LOG(INFO) << "pop success";
  {
    std::lock_guard<std::mutex> lk(odom_lk_);
    odom_meas = odom_datas_;
    TrimOdometryData(time);
  }
  LOG(INFO) << "safe";
  return TrackPose(time, imu_meas, odom_meas);
}

}  // namespace filter
}  // namespace pose_extrapolator
}  // namespace cartographer
