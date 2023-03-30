/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
