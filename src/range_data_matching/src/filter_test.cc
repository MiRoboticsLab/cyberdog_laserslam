/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "range_data_matching/filter/filter.h"

#include <gtest/gtest.h>
#include <glog/logging.h>

TEST(FILTER, ADVANCE) {
  cartographer::mapping::FilterParam param;
  param.position_sigma = 0.1;
  param.rotation_sigma = 0.1;
  param.velocity_sigma = 0.1;
  param.acc_bias_sigma = 0.1;
  param.gyro_bias_sigma = 0.1;
  param.vi_sigma = 0.01;
  param.wi_sigma = 0.01;
  param.thetai_sigma = 0.01;
  param.ai_sigma = 0.01;
  param.update_jocabian = true;
  param.odom_position_sigma = 0.1;
  param.odom_angular_sigma = 0.1;
  param.measure_ba_sigma = 0.1;
  param.measure_bg_sigma = 0.1;
  Eigen::Vector3d extrinsic;
  extrinsic.setZero();
  param.odom_2_imu_extrinsic = extrinsic;
  cartographer::mapping::FilterPtr filter;
  filter.reset(new cartographer::mapping::Filter(param));
  double time = 0.5;
  std::vector<cartographer::mapping::ImuMeasurement> imu_data;
  std::vector<cartographer::mapping::OdomMeasurement> odom_data;
  for (int i = 0; i < 1000; ++i) {
    time += 0.01;
    cartographer::mapping::ImuMeasurement meas;
    meas.acc = cartographer::mapping::V3D(0, 0, -9.81);
    meas.gyro = Eigen::Vector3d(0.01, 0.01, 0.01);
    meas.timestamp = time;
    imu_data.push_back(meas);
  }
  time = 0.0;
  for (int j = 0; j < 50; ++j) {
    time += 0.1;
    cartographer::mapping::OdomMeasurement odom;
    odom.angular.setZero();
    odom.position.setZero();
    odom.timestamp = time;
    odom_data.push_back(odom);
  }
  filter->Advance(100, imu_data, odom_data);
  LOG(INFO) << "newest updated time is: " << filter->newest_updated_time();
  EXPECT_EQ(0, 0);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
