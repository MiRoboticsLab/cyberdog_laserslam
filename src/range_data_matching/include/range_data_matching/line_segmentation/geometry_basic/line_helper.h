/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMAENTATION_GEOMETRY_BASIC_LINE_HELPER_H_
#define RANGE_DATA_MATCHING_LINE_SEGMAENTATION_GEOMETRY_BASIC_LINE_HELPER_H_
#include <cmath>
#include <numeric>

#include <eigen3/Eigen/Core>
#include <glog/logging.h>

#include "range_data_matching/line_segmentation/geometry_basic/point_set.h"
namespace cartographer {
namespace line_extration {
static double dis_to_line(const Eigen::Vector2d& point, double a, double b) {
  return fabs(point.y() - (a + b * point.x()) / sqrt(1 + b * b));
}
static Eigen::Vector2d closest_point_on_line(const Eigen::Vector2d& point,
                                             const Line<double>& line) {
  // projection of point onto line is:
  // P = (a' * p * a) / (a'*a);
  double ax = line.b.x() - line.a.x();
  double ay = line.b.y() - line.a.y();
  double aa = ax * ax + ay * ay;
  double a_dot_p =
      (ax * (point.x() - line.a.x()) + ay * (point.y() - line.a.y())) / aa;
  return Eigen::Vector2d(a_dot_p * ax + line.a.x(), a_dot_p * ay + line.a.y());
}
static double dis_to_line(const Eigen::Vector2d& point,
                          const Line<double>& line) {
  Eigen::Vector2d p = closest_point_on_line(point, line);
  double dis = std::sqrt((p.x() - point.x()) * (p.x() - point.x()) +
                         (p.y() - point.y()) * (p.y() - point.y()));
  return dis;
}

static double slope(const Line<double>& line) {
  LOG(INFO) << line.a.transpose() << " " << line.b.transpose();
  if (fabs(line.a.x() - line.b.x()) < 1e-6) {
    return HUGE_VAL;
  }
  LOG(INFO) << ((line.b.y() - line.a.y()) / (line.b.x() - line.a.x()));
  return ((line.b.y() - line.a.y()) / (line.b.x() - line.a.x()));
}

static bool is_on_line(const Eigen::Vector2d& point, const Line<double>& line) {
  double slope_a = slope(line);
  double b = line.a.y() - slope_a * line.a.x();
  return point.y() == slope_a * point.x() + b;
}
// for cluster accurate, consider a point lie on the left or right of line
// S(p1,p2,p3) is 
static bool is_on_line_left(const Eigen::Vector2d& point,
                            const Line<double>& line) {
  bool is_on_left = true;
  double s_p1_p2_p3 = (line.a.x() - point.x()) * (line.b.y() - point.y()) -
                      (line.a.y() - point.y()) * (line.b.x() - point.x());
  if (s_p1_p2_p3 > 0) {
    is_on_left = true;
  } else {
    is_on_left = false;
  }
  return is_on_left;
}
}  // namespace line_extration
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_LINE_SEGMAENTATION_GEOMETRY_BASIC_LINE_HELPER_H_
