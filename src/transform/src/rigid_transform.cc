/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#include "transform/rigid_transform.h"

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>

namespace cartographer {
namespace transform {

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}
// TimedRigid3d Interpolate(const TimedRigid3d& start, const TimedRigid3d& end,
//                          const common::Time time) {
//   const double duration = common::ToSeconds(end.timestamp - start.timestamp);
//   const double factor = common::ToSeconds(time - start.timestamp) / duration;
//   const Eigen::Vector3d origin =
//       start.pose.translation() +
//       (end.pose.translation() - start.pose.translation()) * factor;
//   const Eigen::Quaterniond rotation =
//       Eigen::Quaterniond(start.pose.rotation())
//           .slerp(factor, Eigen::Quaterniond(end.pose.rotation()));
//   return TimedRigid3d{Rigid3d(origin, rotation), time};
// }
}  // namespace transform
}  // namespace cartographer
