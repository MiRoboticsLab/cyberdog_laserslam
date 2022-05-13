/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "range_data_matching/line_segmentation/transform_pc_2_Image.h"

#include <gtest/gtest.h>
#include <glog/logging.h>

TEST(TRANSFORM_IMAGE, IM_SHOW) {
  cartographer::sensor::Transform2Image transform;
  pcl::PointCloud<pcl::PointXYZ> pc;
  pc.points.resize(400);
  int x = 0;
  int j = -200;
  for (int i = -200; i < 200; ++i) {
    pc.points[x].x = i;
    pc.points[x].y = j;
    pc.points[x].z = 0.0;
    ++x;
    ++j;
  }

  std::vector<Eigen::Vector2d> lines;
  transform.HoughTransform(pc, &lines);
  EXPECT_EQ(1, 1);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}