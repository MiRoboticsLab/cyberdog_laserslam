/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixang@xiaomi.com>
 *
 */
#include "range_data_matching/scan_matching/ceres_scan_matcher_2d.h"

#include <memory>
#include <absl/memory/memory.h>
#include <gtest/gtest.h>

#include "range_data_matching/map/probability_grid.h"
#include "range_data_matching/map/probability_values.h"
#include "sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
class CeresScanMatchingTest : public ::testing::Test {
 protected:
  CeresScanMatchingTest()
      : probability_grid_(
            MapLimits(1., Eigen::Vector2d(10., 10.), CellLimits(20, 20)),
            &conversion_table_) {
    probability_grid_.SetProbability(
        probability_grid_.limits().GetCellIndex(Eigen::Vector2f(-3.5f, 2.5f)),
        kMaxProbability);
    LOG(INFO) << "cell index is: "
              << probability_grid_.limits()
                     .GetCellIndex(Eigen::Vector2f(-3.5f, 2.5f))
                     .transpose();
    Eigen::Vector3f pt(-3.0f, 3.0f, 0.f);
    // transform::Rigid3d rotation =
    //     transform::Rigid3d(Eigen::Vector3d(-0.5, 0.5, 0.0),
    //                        transform::AngleAxisVectorToRotationQuaternion(
    //                            Eigen::Vector3d(0, 0, 0.1)));
    // pt = rotation.cast<float>() * pt;
    point_cloud_.push_back(sensor::RangefinderPoint{pt});
    double occupied_space_weight = 1.0;
    double translation_weight = 0.1;
    double rotation_weight = 1.5;
    bool use_nonmonotonic_steps = false;
    int max_num_iterations = 50;
    int num_threads = 1;
    CeresScanMatchingParam param;
    param.max_num_iterations = max_num_iterations;
    param.num_threads = num_threads;
    param.use_nonmonotonic_steps = use_nonmonotonic_steps;
    param.occupied_space_weight = occupied_space_weight;
    param.rotation_weight = rotation_weight;
    param.translation_weight = translation_weight;
    ceres_scan_matcher_ = absl::make_unique<CeresScanMatcher2D>(param);
  }

  void TestFromInitialPose(const transform::Rigid2d& initial_pose) {
    transform::Rigid2d pose;
    const transform::Rigid2d expected_pose =
        transform::Rigid2d::Translation({-0.3, 0.4});
    // transform::Rigid2d::Translation({-0.5, 0.5});
    ceres::Solver::Summary summary;
    ceres_scan_matcher_->Match(initial_pose.translation(), initial_pose,
                               point_cloud_, probability_grid_, &pose,
                               &summary);
    probability_grid_.GrowLimits(Eigen::Vector2f(26.f, 28.f));
    LOG(INFO) << probability_grid_.limits().cell_limits().num_x_cells;
    LOG(INFO) << probability_grid_.GetProbability(
        probability_grid_.limits().GetCellIndex(Eigen::Vector2f(-2.5f, 1.5f)));
    LOG(INFO) << "initial pose is: " << initial_pose.translation().transpose()
              << "compare to estimated pose: " << pose.translation().transpose()
              << "compare to expect pose is: "
              << expected_pose.translation().transpose() << "pose is: " << pose
              << summary.FullReport();
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
  }

  ValueConversionTables conversion_table_;
  ProbabilityGrid probability_grid_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
};

TEST_F(CeresScanMatchingTest, testPerfectEstimate) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.3, 0.4}));
  // transform::Rigid3d transform(Eigen::Vector3d(-0.6, 0.4, 0.0),
  //                              transform::AngleAxisVectorToRotationQuaternion(
  //                                  Eigen::Vector3d(0.0, 0.0, 0.08)));
  // TestFromInitialPose(transform::Project2D(transform));
}
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
