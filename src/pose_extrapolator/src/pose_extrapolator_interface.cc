/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_extrapolator/pose_extrapolator_interface.h"
#include "pose_extrapolator/pose_extrapolator.h"

namespace cartographer {
namespace pose_extrapolator {
std::unique_ptr<PoseExtraPolatorInterface>
PoseExtraPolatorInterface::CreateWithImuData(
    const FilterParam& param, const std::vector<ImuMeasurement>& imu_datas,
    const std::vector<transform::TimedRigid3d>& initial_poses) {
  if (not param.use_filter) {
    return PoseExtrapolator::InitialWithImu(imu_datas.back(), param);
  } else {
    return PoseExtrapolator::InitialWithPose(initial_poses, param);
  }
}
}  // namespace pose_extrapolator
}  // namespace cartographer
