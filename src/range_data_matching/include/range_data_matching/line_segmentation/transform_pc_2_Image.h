/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_LINE_SEGMENTATION_TRANSFORM_2_IMAGE_H_
#define RANGE_DATA_MATCHING_LINE_SEGMENTATION_TRANSFORM_2_IMAGE_H_
#include <vector>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>

namespace cartographer {
namespace sensor {
struct MaxMin {
  Eigen::Vector2d max;
  Eigen::Vector2d min;
};

class Transform2Image {
 public:
  Transform2Image() {
    max_min_.max << DBL_MIN, DBL_MIN;
    max_min_.min << DBL_MAX, DBL_MAX;
  }
  virtual ~Transform2Image() {}

  void HoughTransform(const pcl::PointCloud<pcl::PointXYZ>& pc,
                      std::vector<Eigen::Vector2d>* lines);

 private:
  void ConvertFromLaserToImage(const pcl::PointCloud<pcl::PointXYZ>& pc,
                               cv::Mat* image);
  void FindMaxMin(const pcl::PointCloud<pcl::PointXYZ>& pc);

  MaxMin max_min_;
};
}  // namespace sensor
}  // namespace cartographer
#endif  // RANGE_DATA_MATCHING_LINE_SEGMENTATION_TRANSFORM_2_IMAGE_H_
