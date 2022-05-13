/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef RANGE_DATA_MATCHING_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define RANGE_DATA_MATCHING_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>

#include "common/param.h"
#include "range_data_matching/map/grid_2d.h"
#include "sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const CeresScanMatchingParam& param);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  ceres::Solver::Options CreateCeresSolverOptions() {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = param_.use_nonmonotonic_steps;
    options.max_num_iterations = param_.max_num_iterations;
    options.num_threads = param_.num_threads;
    return options;
  }
  CeresScanMatchingParam param_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
