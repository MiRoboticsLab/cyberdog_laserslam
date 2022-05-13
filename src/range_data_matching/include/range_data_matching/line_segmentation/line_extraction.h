/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACTION_H_
#define RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACTION_H_
#include <cmath>
#include <numeric>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <glog/logging.h>

#include "range_data_matching/line_segmentation/geometry_basic/point_set.h"
#include "range_data_matching/line_segmentation/geometry_basic/line_helper.h"
#include "range_data_matching/line_segmentation/geometry_basic/least_squre_fit.h"

namespace cartographer {
namespace line_extration {
typedef std::vector<Line<double>> Lines;
class LineExtraction {
 public:
  LineExtraction(double cluster_dist_thre, double max_dist_from_line,
                 double slope_threshold, size_t min_points, int max_iteration)
      : cluster_dist_thre_(cluster_dist_thre),
        max_dist_from_line_(max_dist_from_line),
        slope_threshold_(slope_threshold),
        min_points_(min_points) {
    line_fitter_.reset(new LeastSquare(max_iteration));
  }
  virtual ~LineExtraction() {}

  Lines SplitAndMergeWithPolarCoordinate(
      const std::vector<double>& ranges,
      const std::vector<Eigen::Vector2d>& points);

  // cluster the points with polar coordinate
  void InitialClusterPolar(const std::vector<double>& ranges,
                           std::vector<PointSet<double>>* sets);

 private:
  double cluster_dist_thre_;  // cluster param when init
  double
      max_dist_from_line_;  // max distance from line when do outlier analysis
  double slope_threshold_;  // merge two line with similar slope
  size_t min_points_;       // min points limited within a extracted line
  std::shared_ptr<LeastSquare> line_fitter_;
  /**
   * 0) build line from the set
   * 1) find the split pos
   * 2) recursively call split set in each new set. the first called should be
   * the set with the lower start index to ensure the the ordering of the sets
   * remains monotonic 3) base case: set can not be split, just add it and
   * return
   */
  void SplitSet(const PointSet<double>& set,
                const std::vector<Eigen::Vector2d>& points,
                std::vector<PointSet<double>>& split);

  void SplitSetByDistToLine(const PointSet<double>& set,
                            const std::vector<Eigen::Vector2d>& points,
                            std::vector<PointSet<double>>& splited_set);
  Line<double> BuildLine(const PointSet<double>& set,
                         const std::vector<Eigen::Vector2d>& points) {
    return LeastSquares(set, points);
  }

  Line<double> LeastSquares(const PointSet<double>& set,
                            const std::vector<Eigen::Vector2d>& points);

  Line<double> LineFromLeastSqure(const Eigen::Vector2d& p1,
                                  const Eigen::Vector2d& p2, double avg_x,
                                  double y_intercept, double slope) {
    Line<double> l;
    LOG(INFO) << "slope is: " << slope;
    if (fabs(slope) < 1e-10) {
      LOG(INFO) << "fuck";
      // slope is zero, so line just from p1.x to p2.x
      l.a = Eigen::Vector2d(p1.x(), y_intercept);
      l.b = Eigen::Vector2d(p1.x() + 2.0, y_intercept);
    } else if (slope == INFINITY) {
      LOG(INFO) << "ok";
      l.a = Eigen::Vector2d(avg_x, p1.y());
      l.b = Eigen::Vector2d(avg_x, p1.y() + 2.0);
    } else {
      LOG(INFO) << "point is: " << p1.transpose();
      l.a = Eigen::Vector2d(p1.x(), slope * p1.x() + y_intercept);
      l.b = Eigen::Vector2d(p2.x(), slope * (p2.x()) + y_intercept);
      LOG(INFO) << "fitted line : " << l.a.transpose() << l.b.transpose();
    }
    LOG(INFO) << "point begin end is: " << p1.transpose() << " , "
              << p2.transpose();
    return Line<double>(closest_point_on_line(p1, l),
                        closest_point_on_line(p2, l));
  }

  // max two above finds the place where two consecutive values sit outside the
  // maxmum distance from line threshold. the two values there are the maximum
  // of all such pairs is returned
  int max_two_above(const std::vector<Eigen::Vector2d>& points, int begin,
                    int end, const Line<double>& line);

  std::vector<PointSet<double>> ClusterByDistToLine(
      const PointSet<double>& set, const std::vector<Eigen::Vector2d>& points);

  // 0) Slope of the line is very similar
  // 1) Distance between the lines is close
  void MergeSets(const std::vector<PointSet<double>>& sets,
                 const std::vector<Eigen::Vector2d>& points,
                 std::vector<PointSet<double>>* merged_pts);

  Lines BuildLines(const std::vector<PointSet<double>>& sets,
                   const std::vector<Eigen::Vector2d>& points);
};
}  // namespace line_extration
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_LINE_SEGMENTATION_LINE_EXTRACTION_H_
