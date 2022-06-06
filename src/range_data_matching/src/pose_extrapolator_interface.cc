/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "range_data_matching/filter/filter_helper/imu_based_extrapolator.h"
#include "range_data_matching/filter/filter_helper/pose_extrapolator_interface.h"

namespace cartographer {
namespace mapping {
std::unique_ptr<PoseExtraPolatorInterface>
PoseExtraPolatorInterface::CreateWithImuData(
    const Params& param, const std::vector<ImuMeasurement>& imu_datas,
    const std::vector<transform::TimedRigid3d>& initial_poses) {
  return ImuBasedExtraPolator::InitialWithImu(param, imu_datas, initial_poses);
}
}  // namespace mapping
}  // namespace cartographer
