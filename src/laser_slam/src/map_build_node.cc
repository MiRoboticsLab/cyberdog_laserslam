/**
 * @file map_build_node.cc
 * @author feixiang zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-07-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "laser_slam/map_build_node.h"
using namespace std::chrono_literals;
namespace cartographer {
namespace laser_slam {
MapBuildNode::MapBuildNode(int thread_num)
    : nav2_util::LifecycleNode("map_builder", ""), is_on_active_status_(false),
      is_multi_map_mode_(false), is_map_name_got_(false), save_map_(false),
      map_publish_period_sec_(0.0), last_laser_time_(0), frame_id_(""),
      local_slam_(nullptr), thread_pool_(thread_num), pose_graph_(nullptr),
      pose_recorder_(nullptr), grid_(nullptr), id_{0, 0} {}

MapBuildNode::~MapBuildNode() {}

nav2_util::CallbackReturn
MapBuildNode::on_configure(const rclcpp_lifecycle::State &state) {
    // Get parameters from yaml
    LocalSlamParam param;
    std::string frame_id("");
    this->declare_parameter("frame_id", frame_id);
    this->get_parameter("frame_id", frame_id);
    frame_id_ = frame_id;
    this->declare_parameter("use_imu");
    this->get_parameter("use_imu", param.use_imu);
    this->declare_parameter("use_real_time_correlative_scan_matching");
    this->get_parameter("use_real_time_correlative_scan_matching",
                        param.use_real_time_correlative_scan_matching);
    this->declare_parameter("num_accumulated");
    this->get_parameter("num_accumulated", param.num_accumulated);
    this->declare_parameter("use_filter");
    this->get_parameter("use_filter", param.use_filter);
    this->declare_parameter("gravity_constant");
    this->get_parameter("gravity_constant", param.gravity_constant);
    this->declare_parameter("pose_duration_time");
    this->get_parameter("pose_duration_time", param.pose_duration_time);
    this->declare_parameter("max_time_seconds");
    this->get_parameter("max_time_seconds", param.max_time_seconds);
    this->declare_parameter("max_distance_meters");
    this->get_parameter("max_distance_meters", param.max_distance_meters);
    this->declare_parameter("max_angle_radians");
    this->get_parameter("max_angle_radians", param.max_angle_radians);
    this->declare_parameter("min_range");
    this->get_parameter("min_range", param.min_range);
    this->declare_parameter("max_range");
    this->get_parameter("max_range", param.max_range);
    this->declare_parameter("missing_data_ray_length");
    this->get_parameter("missing_data_ray_length",
                        param.missing_data_ray_length);
    this->declare_parameter("voxel_filter_size");
    this->get_parameter("voxel_filter_size", param.voxel_filter_size);
    this->declare_parameter("min_num_points");
    this->get_parameter("min_num_points", param.min_num_points);
    this->declare_parameter("max_length");
    this->get_parameter("max_length", param.max_length);
    this->declare_parameter("max_range_scale");
    this->get_parameter("max_range_scale", param.max_range_scale);
    this->declare_parameter("submap_param.grid_insert_type");
    this->get_parameter("submap_param.grid_insert_type",
                        param.submap_param.grid_insert_type);
    this->declare_parameter("submap_param.num_range_data");
    this->get_parameter("submap_param.num_range_data",
                        param.submap_param.num_range_data);
    this->declare_parameter("submap_param.grid_type");
    this->get_parameter("submap_param.grid_type", param.submap_param.grid_type);
    this->declare_parameter("submap_param.resolution");
    this->get_parameter("submap_param.resolution",
                        param.submap_param.resolution);
    this->declare_parameter(
        "submap_param.probability_inserter_param.hit_probability");
    this->get_parameter(
        "submap_param.probability_inserter_param.hit_probability",
        param.submap_param.probability_insert_param.hit_probability);
    this->declare_parameter(
        "submap_param.probability_inserter_param.miss_probability");
    this->get_parameter(
        "submap_param.probability_inserter_param.miss_probability",
        param.submap_param.probability_insert_param.miss_probability);
    this->declare_parameter(
        "submap_param.probability_inserter_param.insert_free");
    this->get_parameter(
        "submap_param.probability_inserter_param.insert_free",
        param.submap_param.probability_insert_param.insert_free);
    this->declare_parameter("ceres_scan_matching_param.use_nonmonotonic_steps");
    this->get_parameter("ceres_scan_matching_param.use_nonmonotonic_steps",
                        param.ceres_param.use_nonmonotonic_steps);
    this->declare_parameter("ceres_scan_matching_param.max_num_iterations");
    this->get_parameter("ceres_scan_matching_param.max_num_iterations",
                        param.ceres_param.max_num_iterations);
    this->declare_parameter("ceres_scan_matching_param.num_threads");
    this->get_parameter("ceres_scan_matching_param.num_threads",
                        param.ceres_param.num_threads);
    this->declare_parameter("ceres_scan_matching_param.occupied_space_weight");
    this->get_parameter("ceres_scan_matching_param.occupied_space_weight",
                        param.ceres_param.occupied_space_weight);
    this->declare_parameter("ceres_scan_matching_param.translation_weight");
    this->get_parameter("ceres_scan_matching_param.translation_weight",
                        param.ceres_param.translation_weight);
    this->declare_parameter("ceres_scan_matching_param.rotation_weight");
    this->get_parameter("ceres_scan_matching_param.rotation_weight",
                        param.ceres_param.rotation_weight);
    this->declare_parameter(
        "real_time_correlative_scan_matching_param.linear_search_window");
    this->get_parameter(
        "real_time_correlative_scan_matching_param.linear_search_window",
        param.real_time_param.linear_search_window);
    this->declare_parameter(
        "real_time_correlative_scan_matching_param.angular_search_window");
    this->get_parameter(
        "real_time_correlative_scan_matching_param.angular_search_window",
        param.real_time_param.angular_search_window);
    this->declare_parameter(
        "real_time_correlative_scan_matching_param.translation_delta_cost_"
        "weight");
    this->get_parameter(
        "real_time_correlative_scan_matching_param.translation_delta_cost_"
        "weight",
        param.real_time_param.translation_delta_cost_weight);
    this->declare_parameter(
        "real_time_correlative_scan_matching_param.rotation_delta_cost_weight");
    this->get_parameter(
        "real_time_correlative_scan_matching_param.rotation_delta_cost_weight",
        param.real_time_param.rotation_delta_cost_weight);
    local_slam_.reset(new LocalSlam(param));
    local_slam_param_ = param;

    // Get Parameters of pose graph and init it
    ConstraintBuilderParam constraint_param;
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.min_score");
    this->get_parameter("pose_graph_param.constraint_builder_param.min_score",
                        constraint_param.min_score);
    LOG(INFO) << constraint_param.min_score;
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.log_matches");
    this->get_parameter("pose_graph_param.constraint_builder_param.log_matches",
                        constraint_param.log_matches);
    this->declare_parameter("pose_graph_param.constraint_builder_param.loop_"
                            "closure_rotation_weight");
    this->get_parameter("pose_graph_param.constraint_builder_param.loop_"
                        "closure_rotation_weight",
                        constraint_param.loop_closure_rotation_weight);
    LOG(INFO) << constraint_param.loop_closure_rotation_weight;
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.loop_closure_translation_"
        "weight");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.loop_closure_translation_"
        "weight",
        constraint_param.loop_closure_translation_weight);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.max_constraint_distance");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.max_constraint_distance",
        constraint_param.max_constraint_distance);
    this->declare_parameter("pose_graph_param.constraint_builder_param.ratio");
    this->get_parameter("pose_graph_param.constraint_builder_param.ratio",
                        constraint_param.ratio);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.max_num_"
        "iterations");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.max_num_"
        "iterations",
        constraint_param.ceres_param.max_num_iterations);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.num_threads");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.num_threads",
        constraint_param.ceres_param.num_threads);
    this->declare_parameter("pose_graph_param.constraint_builder_param.ceres_"
                            "params.use_nonmonotonic_"
                            "steps");
    this->get_parameter("pose_graph_param.constraint_builder_param.ceres_"
                        "params.use_nonmonotonic_"
                        "steps",
                        constraint_param.ceres_param.use_nonmonotonic_steps);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.occupied_space_"
        "weight");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.occupied_space_"
        "weight",
        constraint_param.ceres_param.occupied_space_weight);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.translation_"
        "weight");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.ceres_params.translation_"
        "weight",
        constraint_param.ceres_param.translation_weight);
    this->declare_parameter("pose_graph_param.constraint_builder_param.ceres_"
                            "params.rotation_weight");
    this->get_parameter("pose_graph_param.constraint_builder_param.ceres_"
                        "params.rotation_weight",
                        constraint_param.ceres_param.rotation_weight);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
        "depth");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.branch_and_bound_"
        "depth",
        constraint_param.fast_param.branch_and_bound_depth);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
        "window");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.angular_search_"
        "window",
        constraint_param.fast_param.angular_search_window);
    this->declare_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
        "window");
    this->get_parameter(
        "pose_graph_param.constraint_builder_param.fast_param.linear_search_"
        "window",
        constraint_param.fast_param.linear_search_window);
    OptimizationParam optimization_param;
    this->declare_parameter("pose_graph_param.optimization_param.huber_scale");
    this->get_parameter("pose_graph_param.optimization_param.huber_scale",
                        optimization_param.huber_scale);
    LOG(INFO) << optimization_param.huber_scale;
    this->declare_parameter(
        "pose_graph_param.optimization_param.max_num_iterations");
    this->get_parameter(
        "pose_graph_param.optimization_param.max_num_iterations",
        optimization_param.max_num_iterations);
    this->declare_parameter("pose_graph_param.optimization_param.num_threads");
    this->get_parameter("pose_graph_param.optimization_param.num_threads",
                        optimization_param.num_threads);
    this->declare_parameter(
        "pose_graph_param.optimization_param.report_full_summary");
    this->get_parameter(
        "pose_graph_param.optimization_param.report_full_summary",
        optimization_param.report_full_summary);
    PoseGraph2DParam pose_graph_param;
    pose_graph_param.constraint_builder_param = constraint_param;
    pose_graph_param.optimization_param = optimization_param;
    this->declare_parameter("pose_graph_param.optimize_every_n_nodes");
    this->get_parameter("pose_graph_param.optimize_every_n_nodes",
                        pose_graph_param.optimize_every_n_nodes);
    this->declare_parameter("pose_graph_param.max_num_final_iterations");
    this->get_parameter("pose_graph_param.max_num_final_iterations",
                        pose_graph_param.max_num_final_iterations);
    this->declare_parameter("pose_graph_param.matcher_rotation_weight");
    this->get_parameter("pose_graph_param.matcher_rotation_weight",
                        pose_graph_param.matcher_rotation_weight);
    this->declare_parameter("pose_graph_param.matcher_translation_weight");
    this->get_parameter("pose_graph_param.matcher_translation_weight",
                        pose_graph_param.matcher_translation_weight);
    this->declare_parameter("pose_graph_param.max_submaps_maintain");
    this->get_parameter("pose_graph_param.max_submaps_maintain",
                        pose_graph_param.max_submaps_maintain);
    pose_graph_param_ = pose_graph_param;
    pose_graph_.reset(new pose_graph::optimization::BundleAdjustment(
        pose_graph_param, &thread_pool_));

    // publish for visulizing
    pc_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("laser_pose", 10);
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    // data subscriber
    callback_imu_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_laser_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_odometry_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_imu_opt = rclcpp::SubscriptionOptions();
    sub_imu_opt.callback_group = callback_imu_subscriber_;
    auto sub_odom_opt = rclcpp::SubscriptionOptions();
    sub_odom_opt.callback_group = callback_odometry_subscriber_;
    auto sub_laser_opt = rclcpp::SubscriptionOptions();
    sub_laser_opt.callback_group = callback_laser_subscriber_;

    auto node_namespace = this->get_namespace();
    LOG(INFO) << "namespace is: " << node_namespace;
    bool is_namespace = true;
    if (node_namespace == std::string("/")) {
        is_namespace = false;
    }

    std::string imu_topic("");
    this->declare_parameter("imu_topic", imu_topic);
    this->get_parameter("imu_topic", imu_topic);
    if (is_namespace) {
        imu_topic = node_namespace + imu_topic;
    }
    LOG(INFO) << imu_topic;
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&MapBuildNode::ImuCallBack, this, std::placeholders::_1),
        sub_imu_opt);
    std::string odometry_topic("");
    this->declare_parameter("odometry_topic", odometry_topic);
    this->get_parameter("odometry_topic", odometry_topic);
    if (is_namespace) {
        odometry_topic = node_namespace + odometry_topic;
    }
    LOG(INFO) << odometry_topic;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&MapBuildNode::OdomCallback, this, std::placeholders::_1),
        sub_odom_opt);
    std::string laser_topic("");
    this->declare_parameter("laser_scan_topic", laser_topic);
    this->get_parameter("laser_scan_topic", laser_topic);
    if (is_namespace) {
        laser_topic = node_namespace + laser_topic;
    }
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic, rclcpp::SensorDataQoS(),
            std::bind(&MapBuildNode::LaserCallBack, this,
                      std::placeholders::_1),
            sub_laser_opt);

    LOG(INFO) << laser_topic;

    // Extrinsic of 'imu and odom' and 'laser to odom'
    std::vector<double> tmp{0, -1, 0, 0, 0, -1, 1, 0, 0};
    this->declare_parameter("imu_to_odom");
    rclcpp::Parameter tf = this->get_parameter("imu_to_odom");
    tmp = tf.as_double_array();
    transform_ << tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6],
        tmp[7], tmp[8];
    std::vector<double> l_t_o{0, 0, 0};
    this->declare_parameter("laser_to_odom");
    rclcpp::Parameter tf1 = this->get_parameter("laser_to_odom");
    l_t_o = tf1.as_double_array();
    Eigen::Vector3d translation;
    translation << l_t_o[0], l_t_o[1], l_t_o[2];
    laser_t_odom_ = transform::Rigid3d::Translation(translation);

    // pose recorder for vision slam mapping offline
    this->declare_parameter("map_save_path");
    this->get_parameter("map_save_path", map_save_path_);
    std::string pose_save_path = map_save_path_ + "pose.txt";
    pose_recorder_.reset(new PoseRecorder(pose_save_path));

    // Final Map saver
    grid_.reset(new GridForNavigation(
        0.05, param.submap_param.probability_insert_param));

    // Service with Start and Stop Mapping
    std::string start_mapping_service_name;
    this->declare_parameter("start_mapping_service_name");
    this->get_parameter("start_mapping_service_name",
                        start_mapping_service_name);

    start_mapping_service_ = create_service<std_srvs::srv::SetBool>(
        start_mapping_service_name,
        std::bind(&MapBuildNode::StartMappingCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    std::string stop_mapping_service_name;
    this->declare_parameter("stop_mapping_service_name");
    this->get_parameter("stop_mapping_service_name", stop_mapping_service_name);
    stop_service_ = create_service<visualization::srv::Stop>(
        stop_mapping_service_name,
        std::bind(&MapBuildNode::StopMappingCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("is_multi_map_mode");
    this->get_parameter("is_multi_map_mode", is_multi_map_mode_);
    // if is multi map mode, need communicate with miloc
    if (is_multi_map_mode_) {
        std::string map_name_service_name = "get_map_path";
        this->declare_parameter("map_name_service_name");
        this->get_parameter("map_name_service_name", map_name_service_name);
        map_name_client_ =
            this->create_client<std_srvs::srv::Trigger>(map_name_service_name);

        std::string start_mapping_notify_name = "create_map_service";
        this->declare_parameter("start_mapping_notify_name");
        this->get_parameter("start_mapping_notify_name",
                            start_mapping_notify_name);
        start_map_notify_client_ = this->create_client<std_srvs::srv::SetBool>(
            start_mapping_notify_name);

        std::string stop_mapping_notify_name = "finish_map_service";
        this->declare_parameter("stop_mapping_notify_name");
        this->get_parameter("stop_mapping_notify_name",
                            stop_mapping_notify_name);
        stop_map_notify_client_ = this->create_client<std_srvs::srv::SetBool>(
            stop_mapping_notify_name);
    }
    display_ptr_.reset(new GridForDisplay());
    display_ptr_->SetMapPublisher(map_publisher_);
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapBuildNode::on_activate(const rclcpp_lifecycle::State &state) {
    pc_publisher_->on_activate();
    pose_publisher_->on_activate();
    map_publisher_->on_activate();
    //   map_display_->StartThread();
    display_ptr_->StartThread();
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapBuildNode::on_deactivate(const rclcpp_lifecycle::State &state) {
    pc_publisher_->on_deactivate();
    pose_publisher_->on_deactivate();
    map_publisher_->on_deactivate();
    pose_graph_.reset(new pose_graph::optimization::BundleAdjustment(
        pose_graph_param_, &thread_pool_));
    id_data_.clear();
    local_slam_.reset(new LocalSlam(local_slam_param_));
    grid_.reset(new GridForNavigation(
        0.05, local_slam_param_.submap_param.probability_insert_param));
    display_ptr_.reset(new GridForDisplay());
    display_ptr_->SetMapPublisher(map_publisher_);
    LOG(INFO) << "deactive success";
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapBuildNode::on_cleanup(const rclcpp_lifecycle::State &state) {
    pose_graph_.reset();
    pose_recorder_.reset();
    grid_.reset();
    display_ptr_.reset();
    is_on_active_status_ = false;
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapBuildNode::on_shutdown(const rclcpp_lifecycle::State &state) {
    LOG(INFO) << "Shutting Down";
    return nav2_util::CallbackReturn::SUCCESS;
}

bool MapBuildNode::SaveMap(bool save_map, const std::string &map_name) {
    //   map_display_->QuitThread();
    display_ptr_->QuitThread();
    pose_graph_->RunFinalOptimization();
    if (save_map) {
        bool suc = CheckDirectory(map_save_path_);
        if (not suc) {
            LOG(WARNING) << "Directory make have problem";
            return false;
        }
        protos::mapping::proto::PoseGraphHeader header;
        header.set_format_version(1);
        protos::mapping::proto::PoseGraph graph_proto =
            pose_graph_->ToProto(0, true);
        std::string file_name = map_save_path_ + "graph.pbstream";
        stream::ProtoStreamWriter writer(file_name);
        writer.WriteProto(header);
        writer.WriteProto(graph_proto);
        CHECK(writer.Close());
        const auto pose_graph_data = pose_graph_->pose_graph_data();
        std::vector<sensor::RangeData> range_datas;
        auto begin_it = pose_graph_data.trajectory_nodes.BeginOfTrajectory(0);
        auto end_it = pose_graph_data.trajectory_nodes.EndOfTrajectory(0);
        for (; begin_it != end_it; ++begin_it) {
            transform::Rigid3d local_to_global =
                pose_graph_data.trajectory_nodes.at(begin_it->id).global_pose *
                pose_graph_data.trajectory_nodes.at(begin_it->id)
                    .constant_data->local_pose.inverse();
            auto range_data = id_data_.at(begin_it->id);
            auto pc = sensor::TransformRangeData(range_data,
                                                 local_to_global.cast<float>());
            range_datas.push_back(pc);
        }
        auto success = grid_->RayCastByProbability(range_datas);
        if (not success) {
            LOG(WARNING) << "No Range Data";
            return false;
        }
        std::string map_pt = map_save_path_ + map_name;
        grid_->WritePgmByProbabilityGrid(map_pt);
        pose_recorder_->Write(pose_graph_data.trajectory_nodes);
        pose_recorder_->Close();
        LOG(INFO) << "Map Build Success, All Map related data was saved on: "
                  << map_save_path_;
    }
    return true;
}

void MapBuildNode::GetMapPathCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    if (future.get()->success) {
        auto path = future.get()->message;
        LOG(INFO) << "path is: " << path;
        map_save_path_ = path + "/";
        SaveMap(save_map_, "map");
    }
}

void MapBuildNode::StartMappingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        LOG(ERROR)
            << "Received Trigger Mapping request But not in active state, "
               "ignoring!!!";
        return;
    }
    is_on_active_status_ = request->data;
    if (is_multi_map_mode_) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req.get()->data = false;
        while (!start_map_notify_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                LOG(ERROR) << "Interrupted while waiting for the start notify "
                              "service, "
                              "Exiting";
                response->success = false;
                return;
            }
        }
        start_map_notify_client_->async_send_request(req);
    }
    response->success = true;
}

void MapBuildNode::StopMappingCallback(
    const std::shared_ptr<visualization::srv::Stop::Request> request,
    std::shared_ptr<visualization::srv::Stop::Response> response) {
    if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        LOG(ERROR) << "Received Stop Mapping request But not in active state, "
                      "ignoring!!!";
        return;
    }
    is_on_active_status_ = false;
    save_map_ = request->finish;
    bool success = true;
    if (!is_multi_map_mode_) {
        success = SaveMap(request->finish, request->map_name);
    } else {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        while (!map_name_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                LOG(ERROR)
                    << "Interrupted while waiting for the service, Exiting";
                response->success = false;
                return;
            }
        }

        auto result = map_name_client_->async_send_request(
            req, std::bind(&MapBuildNode::GetMapPathCallback, this,
                           std::placeholders::_1));
        LOG(INFO) << "request sended";
        auto req_notify = std::make_shared<std_srvs::srv::SetBool::Request>();
        req_notify.get()->data = false;
        while (!stop_map_notify_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                LOG(ERROR) << "Interrupted while waiting for the notify "
                              "service, Exiting";
                response->success = false;
                return;
            }
        }
        stop_map_notify_client_->async_send_request(req_notify);
    }

    response->success = success;
}

void MapBuildNode::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu) {
    if (not is_on_active_status_)
        return;
    sensor::ImuData imu_meas;
    Eigen::Vector3d acc, gyro;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y,
        imu->linear_acceleration.z;
    gyro << imu->angular_velocity.x, imu->angular_velocity.y,
        imu->angular_velocity.z;
    imu_meas.linear_acceleration = (acc.transpose() * transform_).transpose();
    imu_meas.angular_velocity = (gyro.transpose() * transform_).transpose();
    imu_meas.time =
        common::FromUniversal((imu->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (imu->header.stamp.nanosec + 50) / 100);
    local_slam_->AddImuData(imu_meas);
}

void MapBuildNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    if (not is_on_active_status_)
        return;
    Eigen::Vector3d position;
    Eigen::Quaterniond angular;
    position << odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0;
    angular = Eigen::Quaterniond(
        odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    sensor::OdometryData odom_meas;
    transform::Rigid3d pose(position, angular);
    odom_meas.pose = pose;
    odom_meas.time =
        common::FromUniversal((odom->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (odom->header.stamp.nanosec + 50) / 100);
    local_slam_->AddOdometryData(odom_meas);
}

void MapBuildNode::LaserCallBack(
    const sensor_msgs::msg::LaserScan::SharedPtr laser) {
    if (not is_on_active_status_)
        return;
    std::vector<sensor::RangefinderPoint> points;
    sensor_msgs::msg::PointCloud2 cloud;
    common::Time time =
        common::FromUniversal((laser->header.stamp.sec +
                               common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                                  10000000ll +
                              (laser->header.stamp.nanosec + 50) / 100);
    if (last_laser_time_ > common::ToUniversal(time)) {
        LOG(ERROR) << "Time Back!!!!!!!!!!!!";
        return;
    }
    last_laser_time_ = common::ToUniversal(time);
    last_time_ = time;
    projector_.projectLaser(*laser, cloud, 20.0);

    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(cloud, raw_cloud);
    for (size_t i = 0; i < raw_cloud.points.size(); ++i) {
        sensor::RangefinderPoint pt;
        pt.position.x() = raw_cloud.points[i].x;
        pt.position.y() = raw_cloud.points[i].y;
        pt.position.z() = raw_cloud.points[i].z;
        sensor::RangefinderPoint transformed_pt =
            sensor::RangefinderPoint{laser_t_odom_.cast<float>() * pt.position};
        // to_do: transform pt to tracking frame
        points.push_back(transformed_pt);
    }
    if (local_slam_->LatestPoseExtrapolatorTime() == common::Time::min() ||
        time < local_slam_->LatestPoseExtrapolatorTime()) {
        LOG(INFO) << "return cause by time"
                  << local_slam_->LatestPoseExtrapolatorTime() << "Imu no data";
        return;
    }
    sensor::PointCloud pc(time, points);
    std::unique_ptr<MatchingResult> local_matching_result =
        local_slam_->AddRangeData(pc);
    if (local_matching_result != nullptr) {
        if (local_matching_result->insertion_result != nullptr) {
            const auto node =
                local_matching_result->insertion_result->node_constant_data;
            const auto submaps =
                local_matching_result->insertion_result->insertion_submaps;
            mapping::NodeId node_id = pose_graph_->AddNode(node, 0, submaps);
            id_data_[node_id] = local_matching_result->range_data_in_local;
            display_ptr_->AddSubmap(id_, submaps[0]);
            if (submaps[0]->insertion_finished()) {
                ++id_.submap_index;
            }
        }
        geometry_msgs::msg::PoseStamped pose_pub;
        pose_pub.header.frame_id = frame_id_;
        pose_pub.header.stamp = laser->header.stamp;
        pose_pub.pose.orientation.x =
            local_matching_result->local_pose.rotation().x();
        pose_pub.pose.orientation.y =
            local_matching_result->local_pose.rotation().y();
        pose_pub.pose.orientation.z =
            local_matching_result->local_pose.rotation().z();
        pose_pub.pose.orientation.w =
            local_matching_result->local_pose.rotation().w();
        pose_pub.pose.position.x =
            local_matching_result->local_pose.translation().x();
        pose_pub.pose.position.y =
            local_matching_result->local_pose.translation().y();
        pose_pub.pose.position.z =
            local_matching_result->local_pose.translation().z();
        pose_publisher_->publish(pose_pub);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = laser->header.stamp;
        t.header.frame_id = "vodom";
        t.child_frame_id = "base_link";

        t.transform.translation.x =
            local_matching_result->local_pose.translation().x();
        t.transform.translation.y =
            local_matching_result->local_pose.translation().y();
        t.transform.translation.z =
            local_matching_result->local_pose.translation().z();

        t.transform.rotation.x =
            local_matching_result->local_pose.rotation().x();
        t.transform.rotation.y =
            local_matching_result->local_pose.rotation().y();
        t.transform.rotation.z =
            local_matching_result->local_pose.rotation().z();
        t.transform.rotation.w =
            local_matching_result->local_pose.rotation().w();

        tf_broadcaster_->sendTransform(t);

        geometry_msgs::msg::TransformStamped t1;
        t1.header.stamp = laser->header.stamp;
        t1.header.frame_id = "map";
        t1.child_frame_id = "vodom";
        t1.transform.translation.x = 0.0;
        t1.transform.translation.y = 0.0;
        t1.transform.translation.z = 0.0;

        t1.transform.rotation.x = 0.0;
        t1.transform.rotation.y = 0.0;
        t1.transform.rotation.z = 0.0;
        t1.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t1);

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.points.resize(
            local_matching_result->range_data_in_local.returns.size());
        for (size_t j = 0;
             j < local_matching_result->range_data_in_local.returns.size();
             ++j) {
            pcl_cloud.points[j].x =
                local_matching_result->range_data_in_local.returns[j]
                    .position.x();
            pcl_cloud.points[j].y =
                local_matching_result->range_data_in_local.returns[j]
                    .position.y();
            pcl_cloud.points[j].z =
                local_matching_result->range_data_in_local.returns[j]
                    .position.z();
        }

        sensor_msgs::msg::PointCloud2 pub_pc;
        pcl::toROSMsg(pcl_cloud, pub_pc);
        pub_pc.header.frame_id = frame_id_;
        pc_publisher_->publish(pub_pc);
    }
}

bool MapBuildNode::CheckDirectory(const std::string &path) {
    int result = access(path.c_str(), 0);
    if (result != 0) {
        LOG(INFO) << "Directory not exist, Make dir for it";
        int ret = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (ret != 0) {
            LOG(WARNING) << "Make directory failed";
            return false;
        }
    }
    return true;
}

} // namespace laser_slam
} // namespace cartographer
