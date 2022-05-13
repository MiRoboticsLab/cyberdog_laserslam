/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <glog/logging.h>

#include "range_data_matching/line_segmentation/transform_pc_2_Image.h"

namespace cartographer {
namespace sensor {
void Transform2Image::FindMaxMin(const pcl::PointCloud<pcl::PointXYZ>& pc) {
  for (size_t i = 0; i < pc.points.size(); ++i) {
    double x = pc.points[i].x * 10;
    double y = pc.points[i].y * 10;
    if (x > max_min_.max.x()) max_min_.max.x() = x;
    if (y > max_min_.max.y()) max_min_.max.y() = y;
    if (x < max_min_.min.x()) max_min_.min.x() = x;
    if (y < max_min_.min.y()) max_min_.min.y() = y;
  }
}

void Transform2Image::ConvertFromLaserToImage(
    const pcl::PointCloud<pcl::PointXYZ>& pc, cv::Mat* image) {
  int rows = static_cast<int>(max_min_.max.x() - max_min_.min.x());
  int cols = static_cast<int>(max_min_.max.y() - max_min_.min.y());
  cv::Mat laser_image(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  Eigen::Vector2i offset(static_cast<int>(max_min_.max.x()),
                         static_cast<int>(max_min_.min.y()));
  cv::Vec3b pixel;
  pixel[0] = 0;
  pixel[1] = 0;
  pixel[2] = 255;
  for (size_t i = 0; i < pc.points.size() - 1; ++i) {
    int x = static_cast<int>(pc.points[i].y * 10) + abs(offset.y());
    int y = offset.x() - static_cast<int>(pc.points[i].x * 10);
    laser_image.at<cv::Vec3b>(x, y) = pixel;
  }
  *image = laser_image;
}

void Transform2Image::HoughTransform(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                     std::vector<Eigen::Vector2d>* lines) {
  FindMaxMin(pc);
  cv::Mat image;
  ConvertFromLaserToImage(pc, &image);
  cv::Mat edge;
  cv::Canny(image, edge, 80, 180, 3, false);
  cv::threshold(edge, edge, 170, 255, cv::THRESH_BINARY);

  std::vector<cv::Vec2f> line;
  cv::HoughLines(edge, line, 1, CV_PI / 180, 130, 0, 0);
  for (size_t i = 0; i < line.size(); ++i) {
    double rho = line[i][0];    // distance to original
    double theta = line[i][1];  // angle to x axis
    double a = cos(theta);
    double b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    double length = std::max(image.rows, image.cols);
    cv::Point pt1, pt2;
    pt1.x = x0 + length * (-b);
    pt1.y = y0 + length * (a);
    pt2.x = x0 - length * (-b);
    pt2.y = y0 - length * (a);
    cv::line(image, pt1, pt2, cv::Scalar(0, 255, 0), 1);
  }
  LOG(INFO) << "line size is: " << line.size();
  cv::imshow("edge", image);
  cv::waitKey(10000);
}
}  // namespace sensor
}  // namespace cartographer