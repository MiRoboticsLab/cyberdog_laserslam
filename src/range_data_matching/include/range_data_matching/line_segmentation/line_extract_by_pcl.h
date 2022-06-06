/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACT_BY_PCL_H_
#define RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACT_BY_PCL_H_
#include <vector>

#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <glog/logging.h>

#include "range_data_matching/line_segmentation/geometry_basic/point_set.h"
#include "range_data_matching/line_segmentation/geometry_basic/least_squre_fit.h"
namespace cartographer {
namespace line_extration {
class LineExtractByPCL {
 public:
  LineExtractByPCL(bool set_optimize, double distance_threshold,
                   int method_type, int model_type, int min_points,
                   double max_distance, int max_iteration)
      : min_points_(min_points),
        max_distance_(max_distance),
        line_fitter_(nullptr) {
    seg_.setOptimizeCoefficients(set_optimize);
    if (method_type == 0) {
      seg_.setMethodType(pcl::SAC_RANSAC);
    } else {
      LOG(ERROR) << "still not implemented yet";
    }
    if (model_type == 0) {
      seg_.setModelType(pcl::SACMODEL_LINE);
    }
    seg_.setDistanceThreshold(distance_threshold);
    line_fitter_.reset(new LeastSquare(max_iteration));
  }
  virtual ~LineExtractByPCL() {}

  std::vector<Line<double>> Extract(const std::vector<Eigen::Vector2d>& points);

 private:
  std::vector<std::vector<Eigen::Vector2d>> SplitExtractedPoints(
      const std::vector<std::vector<Eigen::Vector2d>>& points_indices);

  // split a line by cluster, which two points norm distance above max_distance
  // return the index of points where split
  std::vector<int> SplitALine(const std::vector<Eigen::Vector2d>& line);

  Line<double> BuildLineByLeastSqure(
      const std::vector<Eigen::Vector2d>& points);

  int min_points_;
  double max_distance_;
  pcl::ExtractIndices<pcl::PointXYZ> extractor_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  LeastSquarePtr line_fitter_;
};
}  // namespace line_extration
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACT_BY_PCL_H_
