/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_extrapolator/filter/complementary_filter.h"

namespace cartographer {
namespace pose_extrapolator {
namespace filter {
void ComplementaryFilter::GetMeasurement(const Eigen::Vector3d& acc,
                                         Eigen::Quaterniond* meas) {
  double norm = sqrt(acc.x() * acc.x() + acc.y() * acc.y() + acc.z() * acc.z());
  Eigen::Vector3d acc_normed = acc / norm;
  double qw, qx, qy, qz;
  if (acc_normed.z() >= 0) {
    qw = sqrt((acc_normed.z() + 1) * 0.5);
    qx = -acc_normed.y() / (2.0 * qw);
    qy = acc_normed.x() / (2.0 * qw);
    qz = 0;
  } else {
    double x = sqrt((1 - acc_normed.z()) * 0.5);
    qw = -acc_normed.y() / (2.0 * x);
    qx = x;
    qy = 0;
    qz = acc_normed.x() / (2.0 * x);
  }
  Eigen::Quaterniond result(qw, qx, qy, qz);
  result.normalized();
  *meas = result;
}

double ComplementaryFilter::GetAdaptiveGain(double alpha,
                                            const Eigen::Vector3d& acc) {
  double a_mag =
      sqrt(acc.x() * acc.x() + acc.y() * acc.y() + acc.z() * acc.z());
  double error = fabs(a_mag - kGravity) / kGravity;
  double factor;
  double error1 = 0.1;
  double error2 = 0.2;
  double m = 1.0 / (error1 - error2);
  double b = 1.0 - m * error1;
  if (error < error1) {
    factor = 1.0;
  } else if (error < error2) {
    factor = m * error + b;
  } else {
    factor = 0.0;
  }
  return alpha * factor;
}

void ComplementaryFilter::ScaleQuaternion(double gain,
                                          const Eigen::Quaterniond& delta_q,
                                          Eigen::Quaterniond* q) {
  Eigen::Quaterniond result;
  if (delta_q.w() < 0.0) {
    // slerp (spherical linear interpolation)
    double angle = acos(delta_q.w());
    double a = sin(angle * (1.0 - gain)) / sin(angle);
    double b = sin(angle * gain) / sin(angle);
    result = Eigen::Quaterniond((a + b * delta_q.w()), b * delta_q.x(),
                                b * delta_q.y(), b * delta_q.z());
  } else {
    result = Eigen::Quaterniond(((1.0 - gain) + gain * delta_q.w()),
                                gain * delta_q.x(), gain * delta_q.y(),
                                gain * delta_q.z());
  }
  *q = result;
}

void ComplementaryFilter::Update(const Eigen::Quaterniond& predicted_rotation,
                                 const Eigen::Vector3d& acc,
                                 Eigen::Quaterniond* updated_rotation) {
  Eigen::Vector3d recovered_acc;
  // 1. rotate acc into world frame by the inverse predicted rotation
  recovered_acc = predicted_rotation.inverse() * acc;
  // 2. get the measured rotation by acc recovered
  Eigen::Quaterniond q_meas;
  GetMeasurement(recovered_acc, &q_meas);
  // 3. calculate delta q between predict q and measured q
  Eigen::Quaterniond delta_q;
  delta_q = (q_meas * predicted_rotation.inverse()).normalized();
  // 4. interpolate pose between qI and delta_q
  Eigen::Quaterniond scaled_q;
  double gain = GetAdaptiveGain(alpha_, acc);
  ScaleQuaternion(gain, delta_q, &scaled_q);
  // 5. multiply delta q to predict q finally
  Eigen::Quaterniond result;
  result = (predicted_rotation * scaled_q).normalized();
  Eigen::Vector3d axis_predict = MathUtil::quat_2_axis(predicted_rotation);
  Eigen::Vector3d axis_update = MathUtil::quat_2_axis(result);
  Eigen::Vector3d new_result =
      Eigen::Vector3d(axis_update.x(), axis_update.y(), axis_predict.z());
  LOG(INFO) << "axis is: " << new_result.transpose();
  result = MathUtil::axis_2_quat(new_result).normalized();
  *updated_rotation = result;
}
}  // namespace filter
}  // namespace pose_extrapolator
}  // namespace cartographer
