/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <fstream>
#include <iomanip>
#include "range_data_matching/line_segmentation/line_extract_by_pcl.h"

namespace cartographer {
namespace line_extration {
std::vector<Line<double>> LineExtractByPCL::Extract(
    const std::vector<Eigen::Vector2d>& points) {
  std::vector<Line<double>> result;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = points.size();
  cloud->height = 1;
  cloud->points.resize(cloud->height * cloud->width);
  for (size_t i = 0; i < points.size(); ++i) {
    cloud->points[i].x = points[i].x();
    cloud->points[i].y = points[i].y();
    cloud->points[i].z = 0.0;
  }
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  std::vector<std::vector<Eigen::Vector2d>> points_extracted;
  size_t points_num = cloud->points.size();
  while (cloud->size() > 0.05 * points_num) {
    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      LOG(WARNING) << "could not estimate lines";
      break;
    }
    extractor_.setInputCloud(cloud);
    extractor_.setIndices(inliers);
    extractor_.setNegative(false);
    extractor_.filter(*cloud_line);
    std::vector<Eigen::Vector2d> pts;
    for (int j = 0; j < (cloud_line->width * cloud_line->height); ++j) {
      Eigen::Vector2d pt(cloud_line->points[j].x, cloud_line->points[j].y);
      pts.push_back(pt);
    }
    points_extracted.push_back(pts);
    extractor_.setNegative(true);
    extractor_.filter(*cloud_filter);
    cloud.swap(cloud_filter);
  }
  if (points_extracted.empty()) {
    LOG(ERROR) << "some thing nasty gonna happen";
    return result;
  }
  std::vector<std::vector<Eigen::Vector2d>> split_points;
  split_points = SplitExtractedPoints(points_extracted);
  if (split_points.empty()) {
    LOG(ERROR) << "some thing nasty happened";
    return result;
  }
  LOG(INFO) << "splited size is: " << split_points.size();

  for (size_t x = 0; x < split_points.size(); ++x) {
    if (split_points[x].size() > min_points_) {
      result.push_back(BuildLineByLeastSqure(split_points[x]));
    }
  }
  LOG(INFO) << "line size is: " << result.size();
  std::ofstream output2;
  output2.open("/home/zfx/extract.txt", std::ios::out | std::ios::trunc);
  output2 << std::fixed;
  for (size_t k = 0; k < result.size(); ++k) {
    output2 << result[k].a.transpose() << " " << result[k].b.transpose()
            << std::endl;
  }
  output2.close();

  return result;
}

std::vector<std::vector<Eigen::Vector2d>>
LineExtractByPCL::SplitExtractedPoints(
    const std::vector<std::vector<Eigen::Vector2d>>& points_indices) {
  std::vector<std::vector<Eigen::Vector2d>> result;
  for (size_t i = 0; i < points_indices.size(); ++i) {
    std::vector<int> split_pos = SplitALine(points_indices[i]);
    if (split_pos.empty()) {
      result.push_back(points_indices[i]);
    } else {
      int begin = 0;
      for (size_t j = 0; j < split_pos.size(); ++j) {
        std::vector<Eigen::Vector2d> split_points;
        for (int z = begin; z < split_pos[j] + 1; ++z) {
          split_points.push_back(points_indices[i][z]);
        }
        result.push_back(split_points);
        begin = split_pos[j] + 1;
      }
      std::vector<Eigen::Vector2d> last_set;
      for (int z = begin; z < points_indices[i].size(); ++z) {
        last_set.push_back(points_indices[i][z]);
      }
      result.push_back(last_set);  // push back last set
    }
  }
  return result;
}

std::vector<int> LineExtractByPCL::SplitALine(
    const std::vector<Eigen::Vector2d>& line) {
  std::vector<int> result;
  int split_pos = -1;
  int marker = -1;
  for (size_t i = 1; i < line.size(); ++i) {
    if (marker > 0) {
      if (((fabs(line[i].x() - line[marker].x())) > max_distance_) &&
          ((fabs(line[i].y() - line[marker].y())) > max_distance_)) {
        split_pos = marker;
        result.push_back(split_pos);
        marker = -1;
      }
    }
    if (((fabs(line[i].x() - line[i - 1].x())) > max_distance_) &&
        ((fabs(line[i].y() - line[i - 1].y())) > max_distance_)) {
      marker = i - 1;
    }
  }
  if (marker > 0) {
    result.push_back(marker);
  }
  return result;
}

Line<double> LineExtractByPCL::BuildLineByLeastSqure(
    const std::vector<Eigen::Vector2d>& points) {
  double k = 0, b = 0;
  line_fitter_->LineFit(points, &k, &b);
  Line<double> result;
  result.a =
      Eigen::Vector2d(points.begin()->x(), (k * (points.begin()->x()) + b));
  result.b = Eigen::Vector2d(points.end()->x(), k * (points.end()->x()) + b);
  return result;
}

}  // namespace line_extration
}  // namespace cartographer