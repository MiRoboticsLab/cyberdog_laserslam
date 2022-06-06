/**
 * Copyright(c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_POINT_SET_H_
#define RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_POINT_SET_H_
#include <cmath>
#include <vector>
#include <algorithm>

#include <eigen3/Eigen/Core>

namespace cartographer {
namespace line_extration {
template <typename T>
struct Line {
  Line() {}
  Line(const Eigen::Matrix<T, 2, 1>& point_a,
       const Eigen::Matrix<T, 2, 1>& point_b)
      : a(point_a), b(point_b) {}
  template <typename F>
  explicit Line(const Line<F>& copy) : a(copy.a), b(copy.b) {}

  Eigen::Matrix<T, 2, 1> a;
  Eigen::Matrix<T, 2, 1> b;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename U>
struct PointSet {
  PointSet() {}
  PointSet(const std::pair<size_t, size_t>& points) : set(points) {}
  PointSet(const std::pair<size_t, size_t>& points, const Line<U>& line)
      : set(points), line(line) {}
  PointSet(const std::vector<int>& points) : clustered_points(points) {}

  std::pair<size_t, size_t>
      set;  // the set of points represented by index,<begin,end>
  std::vector<int> clustered_points;
  Line<U> line;     // line fit to this set
  int cluster_num;  // initial cluster id
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace line_extration
}  // namespace cartographer
#endif  // RANGE_DATA_MATCHING_LINE_SEGMENTATION_GEOMETRY_BASIC_POINT_SET_H_
