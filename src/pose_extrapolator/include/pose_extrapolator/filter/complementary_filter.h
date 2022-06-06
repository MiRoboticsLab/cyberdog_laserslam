/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_EXTRAPOLATOR_POSE_FILTER_COMPLEMENTARY_FILTER_H_
#define POSE_EXTRAPOLATOR_POSE_FILTER_COMPLEMENTARY_FILTER_H_
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>

#include "pose_extrapolator/filter_helper/math_util.h"
constexpr double kGravity = 9.81;
// Follow the work https://www.mdpi.com/107542 which get good attitude from
// complementary filter
namespace cartographer {
namespace pose_extrapolator {
namespace filter {
class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(double alpha) : alpha_(alpha) {}
  virtual ~ComplementaryFilter() {}

  void Update(const Eigen::Quaterniond& predicted_rotation,
              const Eigen::Vector3d& acc, Eigen::Quaterniond* updated_rotation);

  void GetMeasurement(const Eigen::Vector3d& acc, Eigen::Quaterniond* meas);

 private:
  double GetAdaptiveGain(double alpha, const Eigen::Vector3d& acc);
  void ScaleQuaternion(double gain, const Eigen::Quaterniond& delta_q,
                       Eigen::Quaterniond* q);

  double alpha_;
};
typedef std::shared_ptr<const ComplementaryFilter> ComplementaryFilterConstPtr;
typedef std::shared_ptr<ComplementaryFilter> ComplementaryFilterPtr;
}  // namespace filter
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_POSE_FILTER_COMPLEMENTARY_FILTER_H_
