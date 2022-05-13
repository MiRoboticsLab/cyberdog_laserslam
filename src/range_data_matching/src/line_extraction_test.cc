/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <random>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include "range_data_matching/line_segmentation/geometry_basic/line_helper.h"
#include "range_data_matching/line_segmentation/line_extraction.h"
#include "range_data_matching/line_segmentation/geometry_basic/least_squre_fit.h"

#include <gtest/gtest.h>
#include <glog/logging.h>

// TEST(LINE_EXTRACTION, INIT_CLUSTER) {
//   double dis_thre = 0.05;
//   double max_dist_from_line = 0.1;
//   double slope_threshold = 0.01;
//   int min_points = 8;
//   int max_iteration = 25;
//   cartographer::line_extration::LineExtraction line_extractor(
//       dis_thre, max_dist_from_line, slope_threshold, min_points,
//       max_iteration);
//   std::vector<double> ranges;
//   std::vector<Eigen::Vector2d> points;
//   int angle_increment = 0;
//   std::default_random_engine generator;
//   std::normal_distribution<double> distribution(2, 0.001);

//   for (double i = 0.1; i < 0.2;) {
//     double num = distribution(generator);
//     double x = i;
//     double y = 2 * i + 1;
//     double range = std::sqrt(x * x + y * y);
//     Eigen::Vector2d pt(x, y);
//     points.push_back(pt);
//     ranges.push_back(range);
//     i = i + 0.01;
//   }
//   LOG(INFO) << "size : " << points.size();
//   std::default_random_engine generator1;
//   std::normal_distribution<double> distribution1(-2, 0.01);
//   for (double j = 4.9; j < 5;) {
//     double num1 = distribution1(generator1);
//     double x = j;
//     double y = -2 * j + 3;
//     Eigen::Vector2d pt(x, y);
//     double range1 = std::sqrt(x * x + y * y);
//     points.push_back(pt);
//     ranges.push_back(range1);
//     j = j + 0.01;
//   }
//   LOG(INFO) << "size of points : " << points.size();
//   cartographer::line_extration::Lines lines;
//   lines = line_extractor.SplitAndMergeWithPolarCoordinate(ranges, points);
//   LOG(INFO) << "line 1 a,b is: " << lines[0].a.transpose() << " , "
//             << lines[0].b.transpose();
//   LOG(INFO) << "slope is: " << cartographer::line_extration::slope(lines[0]);
//   //   LOG(INFO) << "set size is: " << sets.size();
//   //   LOG(INFO) << "set 1 is: " << sets[0].set.first << " , " <<
//   //   sets[0].set.second;
//   EXPECT_NEAR(cartographer::line_extration::slope(lines[0]), 2.0, 1e-3);
//   EXPECT_NEAR(cartographer::line_extration::slope(lines[1]), -2.0, 1e-3);
// }

TEST(LINE_EXTRACTION, SLOPE) {
  cartographer::line_extration::Line<double> line;
  line.a = Eigen::Vector2d(1, 3);
  line.b = Eigen::Vector2d(2, 5);
  double slope = cartographer::line_extration::slope(line);
  EXPECT_EQ(slope, 2);
  Eigen::Vector2d pt = Eigen::Vector2d(1, 3.5);
  Eigen::Vector2d closest_pt =
      cartographer::line_extration::closest_point_on_line(pt, line);
  bool is_on = cartographer::line_extration::is_on_line(closest_pt, line);
  EXPECT_TRUE(is_on);
  double dist = cartographer::line_extration::dis_to_line(pt, line);
  LOG(INFO) << "dis to line: " << dist;
}

TEST(LINE_EXTRACTION, DIS_TO_LINE) {
  Eigen::Vector2d p1, p2;
  p1 << 1.0, 1.0;
  p2 << 3.0, 3.0;
  cartographer::line_extration::Line<double> line;
  line.a = p1;
  line.b = p2;
  Eigen::Vector2d point(2.0, 2.5);
  bool is_left;
  is_left = cartographer::line_extration::is_on_line_left(point, line);
  EXPECT_TRUE(is_left);
  Eigen::Vector2d point2(2.0, 1.5);
  is_left = cartographer::line_extration::is_on_line_left(point2, line);
  EXPECT_FALSE(is_left);

  std::ifstream fin("/home/zfx/pcl_line1.txt", std::ios::in);
  if (!fin.is_open()) {
    LOG(INFO) << "file not open";
  }
  std::vector<Eigen::Vector2d> points;
  std::string buf;
  std::string x, y;
  double x_pt, y_pt;
  bool change = false;
  while (std::getline(fin, buf)) {
    for (int i = 0; i < buf.size(); ++i) {
      if (buf[i] == ' ') {
        change = true;
        continue;
      }
      if (not change) {
        x = x + buf[i];
      } else {
        y = y + buf[i];
      }
    }
    change = false;
    x_pt = std::atof(x.c_str());
    y_pt = std::atof(y.c_str());
    Eigen::Vector2d pt(x_pt, y_pt);
    LOG(INFO) << pt.transpose();
    points.push_back(pt);
    x.clear();
    y.clear();
  }
  std::cout << std::endl;
  // std::ifstream fin1("/home/zfx/pcl_line4.txt", std::ios::in);
  // if (!fin.is_open()) {
  //   LOG(INFO) << "file not open";
  // }
  // std::vector<Eigen::Vector2d> points1;
  // std::string buf1;
  // std::string x1, y1;
  // double x_pt1, y_pt1;
  // bool change1 = false;
  // while (std::getline(fin1, buf1)) {
  //   for (int i = 0; i < buf1.size(); ++i) {
  //     if (buf1[i] == ' ') {
  //       change1 = true;
  //       continue;
  //     }
  //     if (not change1) {
  //       x1 = x1 + buf1[i];
  //     } else {
  //       y1 = y1 + buf1[i];
  //     }
  //   }
  //   change1 = false;
  //   x_pt1 = std::atof(x1.c_str());
  //   y_pt1 = std::atof(y1.c_str());
  //   Eigen::Vector2d pt1(x_pt, y_pt);
  //   LOG(INFO) << pt1.transpose();
  //   points1.push_back(pt1);
  //   x1.clear();
  //   y1.clear();
  // }

  // double dis_thre = 0.1;
  // double max_dist_from_line = 0.1;
  // double slope_threshold = 0.001;
  // int min_points = 10;
  // int max_iteration = 25;
  // cartographer::line_extration::LineExtraction line_extractor(
  //     dis_thre, max_dist_from_line, slope_threshold, min_points,
  //     max_iteration);
  // int size = points.size();
  // cartographer::line_extration::PointSet<double> set;
  // set.set.first = 0;
  // set.set.second = size - 1;
  // line_extractor.BuildLine(set, points);
  // std::vector<cartographer::line_extration::PointSet<double>> split_sets;
  cartographer::line_extration::LeastSquare fitter(25);
  double k = 0.0, b = 0.0, k1 = 0.0, b1 = 0.0;
  fitter.LineFit(points, &k, &b);
  // fitter.LineFit(points1, &k1, &b1);
  // // line_extractor.SplitSet(set, points, split_sets);
  // LOG(INFO) << "split set is: " << split_sets.size();
  // std::vector<cartographer::line_extration::PointSet<double>> sets;
  // // line_extractor.MergeSets(split_sets,points,&sets);
  // LOG(INFO) << "merged sets is: " << sets.size();

  EXPECT_EQ(1, 1);
}

TEST(LINE_EXTRACT, SPLIT) {
  std::vector<Eigen::Vector2d> points;
  Eigen::Vector2d p1(1.5, 1.5);
  Eigen::Vector2d p2(1.52, 1.52);
  Eigen::Vector2d p3(1.53, 1.53);
  Eigen::Vector2d p4(2.54, 2.54);
  Eigen::Vector2d p5(2.55, 2.55);
  Eigen::Vector2d p6(3.66, 3.66);
  Eigen::Vector2d p7(3.68, 3.68);
  Eigen::Vector2d p8(3.70, 3.70);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
  points.push_back(p8);
  std::vector<int> result;
  int split_pos = -1;
  int marker = -1;
  for (size_t i = 1; i < points.size(); ++i) {
    LOG(INFO) << "marker pos is: " << marker;
    if (marker > 0) {
      if ((fabs(points[i].norm() - points[marker].norm())) > 0.03) {
        split_pos = marker;
        result.push_back(split_pos);
        marker = -1;
      }
    }
    if (fabs(points[i].norm() - points[i - 1].norm()) > 0.03) {
      marker = i - 1;
    }
  }
  if (marker > 0) {
    result.push_back(marker);
  }
  std::vector<std::vector<Eigen::Vector2d>> pt_sp;
  int begin = 0;
  for (size_t j = 0; j < result.size(); ++j) {
    std::vector<Eigen::Vector2d> split_points;
    for (int z = begin; z < result[j] + 1; ++z) {
      split_points.push_back(points[z]);
    }
    LOG(INFO) << "begin is: " << begin;
    pt_sp.push_back(split_points);
    begin = result[j] + 1;
  }
  std::vector<Eigen::Vector2d> pt_last;
  for (int x = begin; x < points.size(); ++x) {
    pt_last.push_back(points[x]);
  }
  pt_sp.push_back(pt_last);
  for (size_t g = 0; g < pt_sp.size(); ++g) {
    for (size_t n = 0; n < pt_sp[g].size(); ++n) {
      LOG(INFO) << pt_sp[g][n].transpose();
    }
    std::cout << std::endl;
  }
  LOG(INFO) << "result size is: " << result.size();
  EXPECT_EQ(3, pt_sp.size());
  EXPECT_EQ(2, result[0]);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}