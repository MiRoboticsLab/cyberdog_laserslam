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
