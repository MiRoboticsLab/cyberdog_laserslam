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

#include "range_data_matching/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "range_data_matching/map/grid_2d.h"
#include "range_data_matching/scan_matching/cost_functions/occupied_space_cost_function_2d.h"
#include "range_data_matching/scan_matching/cost_functions/rotation_delta_cost_functor_2d.h"
#include "range_data_matching/scan_matching/cost_functions/translation_delta_cost_functor_2d.h"
#include "transform/transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

CeresScanMatcher2D::CeresScanMatcher2D(const CeresScanMatchingParam& param)
    : param_(param), ceres_solver_options_(CreateCeresSolverOptions()) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(param_.occupied_space_weight, 0.);
  if (grid.GetGridType() == GridType::PROBABILITY_GRID) {
    problem.AddResidualBlock(
        CreateOccupiedSpaceCostFunction2D(
            param_.occupied_space_weight /
                std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
        nullptr /* loss function */, ceres_pose_estimate);

  } else {
    CHECK(false) << "undefined grid type";
  }
  CHECK_GT(param_.translation_weight, 0.);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          param_.translation_weight, target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(param_.rotation_weight, 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          param_.rotation_weight, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
