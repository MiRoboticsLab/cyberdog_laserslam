#include "laser_slam/base_data/grid_for_navigation.h"

#include <gtest/gtest.h>
#include <glog/logging.h>

TEST(NAVIGATION_GRID, GRID_CAST) {
  cartographer::laser_slam::GridForNavigation grid(0.05);
  cartographer::sensor::RangeData range_data1, range_data2;
  cartographer::sensor::PointCloud pc1, pc2;
  double x = 20.0;
  for (double i = -15.0; i < 15.0;) {
    cartographer::sensor::RangefinderPoint pt;
    pt.position = Eigen::Vector3d(x, i, 0.0).cast<float>();
    pc1.push_back(pt);
    i += 0.05;
  }
  double x1 = 40.0;
  for (double j = -20; j < 20.0;) {
    cartographer::sensor::RangefinderPoint pt;
    pt.position = Eigen::Vector3d(x1, j, 0.0).cast<float>();
    pc2.push_back(pt);
    j += 0.05;
  }
  range_data1.returns = pc1;
  range_data2.returns = pc2;
  std::vector<cartographer::sensor::RangeData> range_datas;
  range_datas.push_back(range_data1);
  range_datas.push_back(range_data2);
  grid.RayCast(range_datas);
  grid.WritePgm("/home/zfx/test");
  LOG(INFO) << "origin is: " << grid.origin().transpose();
  EXPECT_EQ(1, 1);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
