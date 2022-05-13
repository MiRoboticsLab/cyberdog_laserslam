/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <gtest/gtest.h>
#include <glog/logging.h>

#include "pose_graph/pose_graph_2d.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
// TEST(AddNode, CoreDump) {
//   LocalSlamParam param;
//   param.use_imu = true;
//   param.use_real_time_correlative_scan_matching = true;
//   param.num_accumulated = 1;
//   param.use_filter = false;
//   param.gravity_constant = 10.0;
//   param.pose_duration_time = 0.01;
//   param.max_time_seconds = 5.0;
//   param.max_distance_meters = 0.2;
//   param.max_angle_radians = M_PI / 180;
//   param.min_range = 0.3;
//   param.max_range = 15.0;
//   param.missing_data_ray_length = 5.0;
//   param.voxel_filter_size = 0.025;
//   param.min_num_points = 200;
//   param.max_length = 0.5;
//   param.max_range_scale = 20.0;
//   param.submap_param.grid_insert_type = 0;
//   param.submap_param.grid_type = 0;
//   param.submap_param.num_range_data = 30;
//   param.submap_param.resolution = 0.05;
//   param.submap_param.probability_insert_param.hit_probability = 0.55;
//   param.submap_param.probability_insert_param.miss_probability = 0.49;
//   param.submap_param.probability_insert_param.insert_free = true;
//   param.ceres_param.max_num_iterations = 20;
//   param.ceres_param.num_threads = 1;
//   param.ceres_param.occupied_space_weight = 1.0;
//   param.ceres_param.rotation_weight = 1.0;
//   param.ceres_param.translation_weight = 1.0;
//   param.ceres_param.use_nonmonotonic_steps = false;
//   param.real_time_param.angular_search_window = (M_PI / 180) * 20.0;
//   param.real_time_param.linear_search_window = 0.1;
//   param.real_time_param.rotation_delta_cost_weight = 1e-1;
//   param.real_time_param.translation_delta_cost_weight = 1e-1;
//   BackEndParam back_end_param;
//   back_end_param.local_slam_param = param;
//   back_end_param.thread_num_pool = 4;
//   ConstraintBuilderParam constraint_param;
//   constraint_param.min_score = 0.55;
//   constraint_param.log_matches = true;
//   constraint_param.loop_closure_rotation_weight = 1.e5;
//   constraint_param.loop_closure_translation_weight = 1.1e4;
//   constraint_param.max_constraint_distance = 2.;
//   constraint_param.ratio = 0.3;
//   constraint_param.ceres_param.max_num_iterations = 10;
//   constraint_param.ceres_param.num_threads = 1;
//   constraint_param.ceres_param.use_nonmonotonic_steps = true;
//   constraint_param.ceres_param.occupied_space_weight = 20.;
//   constraint_param.ceres_param.translation_weight = 10.;
//   constraint_param.ceres_param.rotation_weight = 1.;
//   constraint_param.fast_param.branch_and_bound_depth = 7;
//   constraint_param.fast_param.angular_search_window = (M_PI / 180) * 15;
//   constraint_param.fast_param.linear_search_window = 7.;
//   PoseGraph2DParam pose_graph_param;
//   pose_graph_param.constraint_builder_param = constraint_param;
//   OptimizationParam optimization_param;
//   optimization_param.huber_scale = 1e1;
//   optimization_param.max_num_iterations = 50;
//   optimization_param.num_threads = 7;
//   optimization_param.report_full_summary = false;
//   pose_graph_param.optimization_param = optimization_param;
//   pose_graph_param.optimize_every_n_nodes = 90;
//   pose_graph_param.max_num_final_iterations = 200;
//   pose_graph_param.matcher_rotation_weight = 5e2;
//   pose_graph_param.matcher_translation_weight = 1.6e3;
//   back_end_param.pose_graph_param = pose_graph_param;
//   common::ThreadPool thread_pool(1);
//   PoseGraph2D pose_graph(
//       pose_graph_param,
//       std::make_unique<pose_graph::optimization::OptimizationProblem2D>(
//           pose_graph_param.optimization_param),
//       &thread_pool);
//   TrajectoryNode::Data data1 {common::FromSeconds(0.0),Eigen::Quaterniond::Identity(),{},{},{},transform::Rigid3d::Identity()};
  
//   auto constant_data = std::make_shared<const TrajectoryNode::Data>(data1);
// }

}  // namespace optimization

}  // namespace pose_graph
}  // namespace cartographer