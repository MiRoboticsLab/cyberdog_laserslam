/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <iostream>
#include <iomanip>
#include <fstream>

#include "range_data_matching/line_segmentation/line_extraction.h"

namespace cartographer {
namespace line_extration {
void LineExtraction::InitialClusterPolar(const std::vector<double>& ranges,
                                         std::vector<PointSet<double>>* sets) {
  int cur_cluster = 0;
  std::vector<PointSet<double>> result;
  PointSet<double> set(std::make_pair(0, 0));
  std::vector<int> clustered;
  for (size_t x = 1; x < ranges.size(); ++x) {
    clustered.push_back(x);
    if (fabs(ranges[x - 1] - ranges[x]) > cluster_dist_thre_) {
      set.set.second = x;
      set.cluster_num = cur_cluster++;
      set.clustered_points = clustered;
      clustered.clear();
      result.push_back(set);

      set.set.first = x;
      set.set.second = x;
    }
  }
  // push back final set
  set.set.second = ranges.size();
  set.cluster_num = cur_cluster;
  set.clustered_points = clustered;
  result.push_back(set);
  LOG(INFO) << "extracted line num is: "
            << "init sets: " << result.size();
  *sets = result;
}

Line<double> LineExtraction::LeastSquares(
    const PointSet<double>& set, const std::vector<Eigen::Vector2d>& points) {
  std::vector<Eigen::Vector2d> set_points;
  double avg_x = 0;
  if (set.clustered_points.empty()) {
    return Line<double>();
  }
  for (size_t i = 0; i < set.clustered_points.size(); ++i) {
    LOG(INFO) << "clustered num : " << i;
    set_points.push_back(points[set.clustered_points[i]]);
    avg_x += points[set.clustered_points[i]].x();
  }
  avg_x = avg_x / set_points.size();
  double k = 0, b = 0;
  LOG(INFO) << "set points size : " << set_points.size();
  line_fitter_->LineFit(set_points, &k, &b);
  LOG(INFO) << "set points is: " << set_points.size();
  return LineFromLeastSqure(*set_points.begin(), *(set_points.end() - 1), avg_x,
                            b, k);
}

void LineExtraction::SplitSet(const PointSet<double>& set,
                              const std::vector<Eigen::Vector2d>& points,
                              std::vector<PointSet<double>>& split) {
  // follow google style, input param should be const,so just copy a set
  PointSet<double> copy = set;
  int point_num = set.set.second - set.set.second;

  copy.line = BuildLine(copy, points);
  int split_pos =
      max_two_above(points, copy.set.first, copy.set.second, copy.line);
  if (point_num < 2) {
    split_pos = -1;
  }
  if (split_pos > 0) {
    LOG(INFO) << "split pose is: " << split_pos;
    PointSet<double> new_set(std::make_pair(copy.set.first, split_pos));
    SplitSet(new_set, points, split);

    PointSet<double> new_second_set(std::make_pair(split_pos, copy.set.second));
    SplitSet(new_second_set, points, split);
  } else {
    split.push_back(copy);
  }
  LOG(INFO) << "split once";
}

int LineExtraction::max_two_above(const std::vector<Eigen::Vector2d>& points,
                                  int begin, int end,
                                  const Line<double>& line) {
  double max_dist = 0;
  double cur_dist = 0;
  double next_dist = 0;
  int max_pos = -1;
  for (int x = begin; x < end - 1; ++x) {
    next_dist = dis_to_line(points[x + 1], line);
    LOG(INFO) << "dis to line is: " << next_dist;
    if ((fabs(cur_dist) > max_dist_from_line_) &&
        (fabs(next_dist) > max_dist_from_line_) && (next_dist * cur_dist > 0) &&
        (cur_dist > max_dist)) {
      max_dist = cur_dist;
      max_pos = x;
    }
    cur_dist = next_dist;
  }
  return max_pos;
}

void LineExtraction::SplitSetByDistToLine(
    const PointSet<double>& set, const std::vector<Eigen::Vector2d>& points,
    std::vector<PointSet<double>>& splited_set) {
  LOG(INFO) << "incursive go";
  if (set.clustered_points.size() < min_points_) return;
  PointSet<double> copy = set;
  copy.line = BuildLine(copy, points);
  LOG(INFO) << "fuck";
  std::vector<PointSet<double>> clustered_pts =
      ClusterByDistToLine(copy, points);
  LOG(INFO) << "clustered num is: " << clustered_pts.size();
  if (clustered_pts.size() > 1) {
    LOG(INFO) << "size of cluster is: " << clustered_pts.size();
    for (size_t i = 0; i < clustered_pts.size(); ++i) {
      PointSet<double> new_set;
      new_set = clustered_pts[i];
      splited_set.push_back(new_set);
      // SplitSetByDistToLine(new_set, points, splited_set);
    }
  } else if (clustered_pts.empty()) {
    LOG(INFO) << "empty clustered";
  } else {
    LOG(INFO) << "push back ";
    splited_set.push_back(copy);
  }
}

std::vector<PointSet<double>> LineExtraction::ClusterByDistToLine(
    const PointSet<double>& set, const std::vector<Eigen::Vector2d>& points) {
  std::vector<double> distances;
  for (size_t i = 0; i < set.clustered_points.size(); ++i) {
    LOG(INFO) << "clustered points size is: " << set.clustered_points.size();
    double distance = dis_to_line(points[set.clustered_points[i]], set.line);
    if (not is_on_line_left(points[set.clustered_points[i]], set.line)) {
      distance = -distance;
    }
    distances.push_back(distance);
  }
  LOG(INFO) << "distance culculated finished";
  std::vector<int> origin_cluster_sets;
  std::vector<int> left_cluster_sets;
  std::vector<int> right_cluster_sets;
  for (size_t j = 0; j < distances.size(); ++j) {
    if (fabs(distances[j]) < max_dist_from_line_) {
      LOG(INFO) << "distance to line is: " << distances[j];
      origin_cluster_sets.push_back(set.clustered_points[j]);
    } else if (distances[j] < 0) {
      LOG(INFO) << "distance to line is: " << distances[j];
      right_cluster_sets.push_back(set.clustered_points[j]);
    } else {
      LOG(INFO) << "distance to line is: " << distances[j];
      left_cluster_sets.push_back(set.clustered_points[j]);
    }
  }
  std::vector<PointSet<double>> result;
  if (origin_cluster_sets.size() > min_points_) {
    LOG(INFO) << "min points is: " << min_points_;
    PointSet<double> p1;
    p1.clustered_points = origin_cluster_sets;
    result.push_back(p1);
  }
  if (left_cluster_sets.size() > min_points_) {
    PointSet<double> p2;
    p2.clustered_points = left_cluster_sets;
    result.push_back(p2);
  }
  if (right_cluster_sets.size() > min_points_) {
    PointSet<double> p3;
    p3.clustered_points = right_cluster_sets;
    result.push_back(p3);
  }
  LOG(INFO) << "cluster over";
  return result;
}

void LineExtraction::MergeSets(const std::vector<PointSet<double>>& sets,
                               const std::vector<Eigen::Vector2d>& points,
                               std::vector<PointSet<double>>* merged_pts) {
  std::vector<PointSet<double>> result_sets;
  if (sets.empty()) return;
  PointSet<double> cur_set(sets[0].set);
  size_t merge_start = sets[0].set.first;
  for (size_t x = 1; x < sets.size(); ++x) {
    LOG(INFO) << "slope is: " << fabs(slope(sets[x - 1].line)) << "and "
              << fabs(slope(sets[x].line));
    if (fabs(slope(sets[x - 1].line) - slope(sets[x].line)) >
            slope_threshold_ ||
        ((fabs(dis_to_line(sets[x].line.a, sets[x - 1].line)) >
          max_dist_from_line_) &&
         (fabs(dis_to_line(sets[x].line.b, sets[x - 1].line)) >
          max_dist_from_line_))) {
      cur_set.set.first = merge_start;
      cur_set.set.second = sets[x - 1].set.second;
      cur_set.line = BuildLine(cur_set, points);
      result_sets.push_back(cur_set);
      merge_start = sets[x].set.first;
    }
  }
  cur_set.set.first = merge_start;
  cur_set.set.second = sets[sets.size() - 1].set.second;
  result_sets.push_back(cur_set);
  LOG(INFO) << "extracted line num is: merged sets: " << result_sets.size();
  *merged_pts = result_sets;
}
/**
 * TO_DO(feixiang zeng): merge the line which less points to other lines
 */
Lines LineExtraction::BuildLines(const std::vector<PointSet<double>>& sets,
                                 const std::vector<Eigen::Vector2d>& points) {
  Lines extracted(sets.size());
  int num = 0;
  for (size_t i = 0; i < sets.size(); ++i) {
    if (sets[i].set.second - sets[i].set.first > min_points_) {
      extracted.push_back(sets[i].line);
    }
  }
  return extracted;
}

Lines LineExtraction::SplitAndMergeWithPolarCoordinate(
    const std::vector<double>& ranges,
    const std::vector<Eigen::Vector2d>& points) {
  std::vector<PointSet<double>> sets;
  InitialClusterPolar(ranges, &sets);
  std::vector<PointSet<double>> split_sets;
  Lines extracted_lines;
  for (size_t x = 0; x < sets.size(); ++x) {
    // Line<double> line = BuildLine(sets[x], points);
    SplitSetByDistToLine(sets[x], points, split_sets);
  }
  LOG(INFO) << "split over"
            << "size is: " << split_sets.size();
  std::ofstream output;
  std::string name;
  for (size_t i = 0; i < sets.size(); ++i) {
    name = "/home/zfx/cluster" + std::to_string(i) + ".txt";
    output.open(name, std::ios::out | std::ios::trunc);
    if ((sets[i].set.second - sets[i].set.first) > 5) {
      for (int j = sets[i].set.first; j < sets[i].set.second; ++j) {
        output << std::setprecision(8) << points[j].x() << " " << points[j].y()
               << std::endl;
      }
    }
    output.close();
  }
  LOG(INFO) << "extracted line num is: split set is: " << split_sets.size();
  // sets.clear();
  // MergeSets(split_sets, points, &sets);
  //   Lines extracted_lines(sets.size());
  //   extracted_lines = BuildLines(sets, points);

  for (size_t i = 0; i < split_sets.size(); ++i) {
    extracted_lines.push_back(BuildLine(split_sets[i], points));
  }
  // extracted_lines = BuildLines(split_sets, points);
  return extracted_lines;
}
}  // namespace line_extration
}  // namespace cartographer
