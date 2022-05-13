/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_LEAST_SQUARE_FIT_H_
#define RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_LEAST_SQUARE_FIT_H_
#include <vector>
#include <memory>

#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>

namespace cartographer {
namespace line_extration {
struct LineResidual {
  LineResidual(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const k, const T* const b, T* residual) const {
    residual[0] = y_ - k[0] * x_ - b[0];
    return true;
  }

 private:
  const double x_;
  const double y_;
};

class LeastSquare {
 public:
  LeastSquare(int max_iterations) : max_iterations_(max_iterations) {}
  virtual ~LeastSquare() {}

  void LineFit(const std::vector<Eigen::Vector2d>& points, double* k,
               double* b) {
    double k_c = *k;
    double b_c = *b;
    for (size_t i = 0; i < points.size(); ++i) {
      problem_.AddResidualBlock(
          new ceres::AutoDiffCostFunction<LineResidual, 1, 1, 1>(
              new LineResidual(points[i].x(), points[i].y())),
          NULL, &k_c, &b_c);
    }
    LOG(INFO) << "dskg";
    ceres::Solver::Options options;
    options.max_num_iterations = max_iterations_;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    // LOG(INFO) << summary.BriefReport();
    LOG(INFO) << "k is: " << k_c << "b is: " << b_c;
    *k = k_c;
    *b = b_c;
  }

 private:
  int max_iterations_;
  ceres::Problem problem_;
};
typedef std::shared_ptr<const LeastSquare> LeastSquareConstPtr;
typedef std::shared_ptr<LeastSquare> LeastSquarePtr;

}  // namespace line_extration
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_LEAST_SQUARE_FIT_H_