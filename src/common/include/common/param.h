/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef RANGE_DATA_MATCHING_COMMON_PARAM_H_
#define RANGE_DATA_MATCHING_COMMON_PARAM_H_
#include <eigen3/Eigen/Core>
namespace cartographer {
struct ProbabilityInserterParam {
    double hit_probability;
    double miss_probability;
    bool insert_free;
};

/**
 * grid insert type: 0(probability), 1(tsdf)
 */
struct SubMapParam {
    int grid_insert_type; // grid insert type (probability/ tsdf)
    int num_range_data;   // the num of range data which insert within a submap
    int grid_type;        // grid type (probability/tsdf)
    float resolution;     // resolution of map
    ProbabilityInserterParam probability_insert_param;
};

struct CeresScanMatchingParam {
    bool use_nonmonotonic_steps;
    int max_num_iterations;
    int num_threads;
    double occupied_space_weight;
    double translation_weight;
    double rotation_weight;
};

struct FastCorrelativeScanMatcherParam {
    int branch_and_bound_depth;
    double linear_search_window;
    double angular_search_window;
};

struct RealTimeCorrelativeScanMatcherParam {
    double linear_search_window;
    double angular_search_window;
    double translation_delta_cost_weight;
    double rotation_delta_cost_weight;
};

struct MotionFilterParam {
    double max_time_seconds;
    double max_distance_meters;
    double max_angle_radians;
};

struct OptimizationParam {
    double huber_scale;
    double odometry_traslation_weight;
    double odometry_rotation_weight;
    bool use_nonmonotonic_steps = false;
    int max_num_iterations;
    int num_threads;
    bool report_full_summary = false;
};

struct FilterParam {
    int integration_type; // 0 is midpoint
    bool use_filter;
    bool use_odometry_update_ekf;
    double alpha;
    double gravity_constant;
    double timed_pose_queue_duration;
    double position_sigma;
    double rotation_sigma;
    double velocity_sigma;
    double acc_bias_sigma;
    double gyro_bias_sigma;
    double vi_sigma;
    double wi_sigma;
    double thetai_sigma;
    double ai_sigma;
    bool update_jocabian;
    double odom_position_sigma;
    double odom_angular_sigma;
    double measure_ba_sigma;
    double measure_bg_sigma;
    double complementary_filter_alpha;
    double pose_duration_time;
    double imu_gravity_time_constant;
    Eigen::Vector3d odom_2_imu_extrinsic;
};

struct LocalSlamParam {
    bool use_imu;
    bool use_real_time_correlative_scan_matching;
    // nums laser frame accumulated
    int num_accumulated;

    // pose extrapolator param
    bool use_filter;
    double gravity_constant;
    double pose_duration_time;

    // motion filter param
    double max_time_seconds;
    double max_distance_meters;
    double max_angle_radians;

    // Laser Sensor param
    double min_range;
    double max_range;
    double missing_data_ray_length;

    // voxel filter
    double voxel_filter_size;
    float min_num_points;
    float max_length;
    float max_range_scale;

    // submap param
    SubMapParam submap_param;

    // ceres scan matching param
    CeresScanMatchingParam ceres_param;

    // real time scan matching
    RealTimeCorrelativeScanMatcherParam real_time_param;
};

struct ConstraintBuilderParam {
    float min_score;
    float reloc_min_score;
    CeresScanMatchingParam ceres_param;
    FastCorrelativeScanMatcherParam fast_param;
    bool log_matches;
    double loop_closure_translation_weight;
    double loop_closure_rotation_weight;
    double max_constraint_distance;
    double max_reloc_constraint_distance;
    double max_find_reloc_constraint_distance;
    double ratio;
};

struct PoseGraph2DParam {
    ConstraintBuilderParam constraint_builder_param;
    double matcher_translation_weight;
    double matcher_rotation_weight;
    int optimize_every_n_nodes;
    int max_num_final_iterations;
    int max_submaps_maintain;
    bool need_vision_verify;
    bool localization_mode;
    OptimizationParam optimization_param;
};

struct LocalizationParam {
    std::string channel_name;
    std::string pbstream_file_path;
    int thread_num_pool;
    LocalSlamParam local_slam_param;
    PoseGraph2DParam pose_graph_param;
};

struct BackEndParam {
    PoseGraph2DParam pose_graph_param;
    LocalSlamParam local_slam_param;
    int thread_num_pool;
};
} // namespace cartographer

#endif // RANGE_DATA_MATCHING_COMMON_PARAM_H_