/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <chrono>
#include <math.h>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include "laser_slam/local_slam.h"
#include "rviz_display/probability_grid_conversion.h"

constexpr double kAngleIncrement = M_PI / 360;
constexpr double kDistanceIncrement = 0.05;
namespace cartographer {
namespace laser_slam {
TEST(LOCAL_SLAM, SCAN_MATCHING) {
  LocalSlamPtr local_slam;
  LocalSlamParam param;
  param.use_imu = true;
  param.use_real_time_correlative_scan_matching = true;
  param.num_accumulated = 0;
  param.use_filter = false;
  param.gravity_constant = 10.0;
  param.pose_duration_time = 0.01;
  param.max_time_seconds = 5.0;
  param.max_distance_meters = 0.2;
  param.max_angle_radians = M_PI / 180;
  param.min_range = 0.3;
  param.max_range = 15.0;
  param.missing_data_ray_length = 5.0;
  param.voxel_filter_size = 0.025;
  param.min_num_points = 100;
  param.max_length = 0.5;
  param.max_range_scale = 20.0;
  param.submap_param.grid_insert_type = 0;
  param.submap_param.grid_type = 0;
  param.submap_param.num_range_data = 20;
  param.submap_param.resolution = 0.05;
  param.submap_param.probability_insert_param.hit_probability = 0.55;
  param.submap_param.probability_insert_param.miss_probability = 0.49;
  param.submap_param.probability_insert_param.insert_free = true;
  param.ceres_param.max_num_iterations = 20;
  param.ceres_param.num_threads = 1;
  param.ceres_param.occupied_space_weight = 40.0;
  param.ceres_param.rotation_weight = 10.0;
  param.ceres_param.translation_weight = 10.0;
  param.ceres_param.use_nonmonotonic_steps = false;
  param.real_time_param.angular_search_window = (M_PI / 180) * 20.0;
  param.real_time_param.linear_search_window = 0.1;
  param.real_time_param.rotation_delta_cost_weight = 1e-1;
  param.real_time_param.translation_delta_cost_weight = 1e-1;
  local_slam.reset(new LocalSlam(param));
  std::vector<sensor::RangefinderPoint> points;
  float i = -15.f;
  for (; i < 15.f;) {
    points.push_back(sensor::RangefinderPoint{Eigen::Vector3f(10.0, i, 0)});
    i = i + 0.5f;
  }
  sensor::PointCloud pc(common::FromUniversal(0), points);
  sensor::ImuData im{common::FromUniversal(0), Eigen::Vector3d(0, 0, 10),
                     Eigen::Vector3d(0, 0, 0)};
  std::vector<sensor::RangefinderPoint> new_points;
  local_slam->AddImuData(im);
  local_slam->AddRangeData(pc);
  double angle = (M_PI / 180) * 10.0;
  transform::Rigid3d rotation;
  LOG(INFO) << transform::AngleAxisVectorToRotationQuaternion(
                   Eigen::Vector3d(0, 0, 0.15))
                   .matrix();

  rotation = transform::Rigid3d(Eigen::Vector3d(0.0, 0.0, 0.0),
                                transform::AngleAxisVectorToRotationQuaternion(
                                    Eigen::Vector3d(0, 0, -0.15)));
  LOG(INFO) << "rotation is: " << rotation;
  for (size_t j = 0; j < pc.points().size(); ++j) {
    sensor::RangefinderPoint pt = rotation.cast<float>() * pc.points()[j];
    new_points.push_back(pt);
  }
  common::Time time = pc.time() + common::FromSeconds(0.05);
  sensor::PointCloud new_pc(time, new_points);
  // local_slam->AddPose(transform::TimedRigid3d{rotation.inverse(), time});
  std::unique_ptr<MatchingResult> result = local_slam->AddRangeData(new_pc);
  if (result != nullptr) {
    LOG(INFO) << transform::RotationQuaternionToAngleAxisVector(
                     result->local_pose.rotation())
                     .transpose();
  }
  transform::Rigid3d rotation1;
  rotation1 = transform::Rigid3d::Rotation(
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(0, 0, -0.25)));
  LOG(INFO) << "rotation is: " << rotation1;
  std::vector<sensor::RangefinderPoint> new_points1;
  for (size_t z = 0; z < pc.points().size(); ++z) {
    sensor::RangefinderPoint pt = rotation1.cast<float>() * pc.points()[z];
    new_points1.push_back(pt);
  }
  common::Time time1 = pc.time() + common::FromSeconds(0.1);
  sensor::PointCloud pc2(time1, new_points1);
  // local_slam->AddPose(transform::TimedRigid3d{rotation1.inverse(), time1});
  std::chrono::steady_clock::time_point start_time =
      std::chrono::steady_clock::now();
  std::unique_ptr<MatchingResult> result1 = local_slam->AddRangeData(pc2);
  std::chrono::steady_clock::time_point end_time =
      std::chrono::steady_clock::now();
  std::chrono::duration<double> duration = end_time - start_time;
  LOG(INFO) << "scan match consume: " << duration.count() * 1000;
  if (result1 != nullptr) {
    LOG(INFO) << transform::RotationQuaternionToAngleAxisVector(
                     result1->local_pose.rotation())
                     .transpose();
  }
  transform::Rigid3d rotation2;
  rotation2 = transform::Rigid3d::Rotation(
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(0, 0, -0.35)));
  LOG(INFO) << "rotation is: " << rotation1;
  std::vector<sensor::RangefinderPoint> new_points2;
  for (size_t z = 0; z < pc.points().size(); ++z) {
    sensor::RangefinderPoint pt = rotation2.cast<float>() * pc.points()[z];
    new_points2.push_back(pt);
  }
  common::Time time2 = pc.time() + common::FromSeconds(0.2);
  sensor::PointCloud pc3(time2, new_points2);
  std::unique_ptr<MatchingResult> result2 = local_slam->AddRangeData(pc3);
  if (result2 != nullptr) {
    LOG(INFO) << transform::RotationQuaternionToAngleAxisVector(
                     result2->local_pose.rotation())
                     .transpose();
  }
  //   int index = 0;
  //   for (double angle = M_PI / 180; angle < M_PI;) {
  //     transform::Rigid3d rotation;
  //     rotation.Rotation(transform::AngleAxisVectorToRotationQuaternion(
  //         Eigen::Vector3d(0, 0, angle)));
  //     common::Time time = pc.time() + common::FromSeconds(0.05 * index);
  //     ++index;
  //     std::vector<sensor::RangefinderPoint> points = pc.points();
  //     for (size_t j = 0; j < points.size(); ++j) {
  //       sensor::RangefinderPoint hit = rotation.cast<float>() * points[j];
  //       new_points.push_back(hit);
  //     }
  //     sensor::PointCloud new_pc(time, new_points);
  //     std::unique_ptr<MatchingResult> result =
  //     local_slam->AddRangeData(new_pc); if (result != nullptr) {
  //       LOG(INFO) << "pose is: " << result->local_pose;
  //     }
  //     new_points.clear();
  //     angle = angle + kAngleIncrement;
  //   }
}
}  // namespace laser_slam
}  // namespace cartographer

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}