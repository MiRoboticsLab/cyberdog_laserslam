/*
 * Copyright 2016 The Cartographer Authors
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
#include "pose_graph/cost_function/spa_cost_function.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
namespace {
class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(const Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error =
        ScaleError(ComputeUnscaledError(
                       transform::Project2D(observed_relative_pose_.zbar_ij),
                       start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  Pose observed_relative_pose_;

  template <typename T>
  std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                              double translation_weight,
                              double rotation_weight) const {
    // clang-format off
      return {{
          error[0] * translation_weight,
          error[1] * translation_weight,
          error[2] * rotation_weight
      }};
    // clang-format on
  }

  template <typename T>
  std::array<T, 3> ComputeUnscaledError(const transform::Rigid2d& relative_pose,
                                        const T* const start,
                                        const T* const end) const {
    const T cos_theta_i = cos(start[2]);
    const T sin_theta_i = sin(start[2]);
    const T delta_x = end[0] - start[0];
    const T delta_y = end[1] - start[1];
    const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                    -sin_theta_i * delta_x + cos_theta_i * delta_y,
                    end[2] - start[2]};
    return {{T(relative_pose.translation().x()) - h[0],
             T(relative_pose.translation().y()) - h[1],
             common::NormalizeAngleDifference(
                 T(relative_pose.rotation().angle()) - h[2])}};
  }
};
}  // namespace

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const Pose& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer
