/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/localization.h"
#include "protos/proto_stream.h"

namespace cartographer {
namespace laser_slam {
Localization::Localization(const LocalizationParam &param)
    : trajectory_id_(0), reloc_id_(0), is_reloc_(false), start_(true),
      state_(State::INITIALIZATION), param_(param), reloc_pose_(nullptr),
      local_slam_(nullptr), thread_pool_(param.thread_num_pool),
      pose_graph_(nullptr), map_loader_(nullptr) {}
Localization::~Localization() {}

bool Localization::Stop() {
    start_ = false;
    while (pose_graph_->is_working()) {
        LOG(INFO) << "Reloc thread still in working";
        usleep(1000);
    }
    return true;
}

void Localization::SetRelocPublisher(
    const rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
        &publisher) {
    reloc_publisher_ = publisher;
}

void Localization::SetCallback(const PosePcCallback &callback) {
    pose_pc_callback_ = callback;
}

bool Localization::Initialize() {
    pose_graph_.reset(new pose_graph::optimization::BundleAdjustment(
        param_.pose_graph_param, &thread_pool_));
    LOG_IF(ERROR, param_.pbstream_file_path.empty())
        << "Pbstream Path Wrong, which is: " << param_.pbstream_file_path;
    stream::ProtoStreamReader reader(param_.pbstream_file_path);
    stream::ProtoStreamDeserializer deserializer(&reader);
    map_loader_ = std::make_shared<MapLoader>(deserializer);
    InitialPoseGraph();
    LOG(INFO) << "Initialize done";
    state_ = State::RELOCATION;
    return true;
}

void Localization::AddRangeData(const sensor::PointCloud &timed_point_cloud) {
    transform::Rigid3d optimized_pose;
    std::unique_ptr<MatchingResult> matching_result;
    if (reloc_pose_ == nullptr) {
        state_ = State::RELOCATING;
    }
    if (state_ == State::RELOCATION_SUCESS) {
        // verify reloc pose by scan matching
        LOG(INFO) << "Got Vision Reloc Pose";
        transform::Rigid3d pose = reloc_pose_->pose;
        sensor::RangeData range_data = sensor::RangeData{{}, {}, {}};
        Eigen::Vector3f origin = pose.cast<float>().translation();
        for (size_t i = 0; i < timed_point_cloud.size(); ++i) {
            sensor::RangefinderPoint hit_in_local = sensor::RangefinderPoint{
                pose.cast<float>() * timed_point_cloud[i].position};
            const Eigen::Vector3f delta = hit_in_local.position - origin;
            const float range = delta.norm();
            if (range >= param_.local_slam_param.min_range) {
                if (range <= param_.local_slam_param.max_range) {
                    range_data.returns.push_back(hit_in_local);
                } else {
                    hit_in_local.position =
                        origin +
                        param_.local_slam_param.missing_data_ray_length /
                            range * delta;
                    range_data.misses.push_back(hit_in_local);
                }
            }
        }
        range_data.origin = pose.translation().cast<float>();
        // transform range data to gravity aligned
        const transform::Rigid3f gravity_algined_pose =
            transform::Rigid3f::Rotation(pose.rotation().cast<float>()) *
            pose.inverse().cast<float>();
        const sensor::RangeData gravity_aligned_data =
            sensor::TransformRangeData(range_data, gravity_algined_pose);
        sensor::RangeData aligned_point_cloud{
            gravity_aligned_data.origin,
            sensor::VoxelFilter(gravity_aligned_data.returns,
                                param_.local_slam_param.voxel_filter_size),
            sensor::VoxelFilter(gravity_aligned_data.misses,
                                param_.local_slam_param.voxel_filter_size)};
        sensor::AdaptiveVoxelFilterParam voxel;
        voxel.max_length = param_.local_slam_param.max_length;
        voxel.max_range = param_.local_slam_param.max_range;
        voxel.min_num_points = param_.local_slam_param.min_num_points;
        const sensor::PointCloud &filtered_point_cloud =
            sensor::AdaptiveVoxelFilter(aligned_point_cloud.returns, voxel);
        std::shared_ptr<const pose_graph::optimization::TrajectoryNode::Data>
            constant_data = std::make_shared<
                const pose_graph::optimization::TrajectoryNode::Data>(
                pose_graph::optimization::TrajectoryNode::Data{
                    timed_point_cloud.time(), pose.rotation(),
                    filtered_point_cloud, pose});
        std::shared_ptr<const pose_graph::optimization::TrajectoryNode> node =
            std::make_shared<const pose_graph::optimization::TrajectoryNode>(
                pose_graph::optimization::TrajectoryNode{constant_data, pose});
        is_reloc_ = pose_graph_->FindRelocConstraints(node, trajectory_id_,
                                                      &optimized_pose);
        std_msgs::msg::Int32 reloc_result;
        if (!is_reloc_) {
            LOG(WARNING)
                << "Reloc falied because failure of scan match to old map";
            state_ = State::RELOCATION;
            reloc_pose_ = nullptr;
            reloc_result.data = 100;
            reloc_publisher_->publish(reloc_result);
            return;
        } else {
            reloc_result.data = 0;
            reloc_publisher_->publish(reloc_result);
            LOG(INFO) << "Reloc success, Connect new node to old pose graph";
            first_reloc_pose_ = optimized_pose;
            local_slam_.reset(new LocalSlam(param_.local_slam_param));
            std::vector<transform::TimedRigid3d> initial_poses;
            // Initialize phase, the dog do not run anywhere, the pose from last
            // two or three seconds could view as pose now
            initial_poses.push_back(transform::TimedRigid3d{
                optimized_pose, timed_point_cloud.time()});
            local_slam_->InitialExtrapolatorWithPoses(initial_poses);
            matching_result = local_slam_->AddRangeData(timed_point_cloud);
            if (matching_result != nullptr) {
                if (matching_result->insertion_result != nullptr) {
                    const auto submaps =
                        matching_result->insertion_result->insertion_submaps;
                    auto id =
                        pose_graph_->AddFirstRelocNode(trajectory_id_, submaps);
                }
            }
            state_ = State::LOCATION;
            return;
        }
    } else if (state_ == State::RELOCATING) {
        LOG_EVERY_N(INFO, 5) << "Relocating, Please wait moment";
        return;
    } else if (state_ == State::LOCATION) {
        // Location phase
        matching_result = local_slam_->AddRangeData(timed_point_cloud);
        sensor::RangeData range_data_callback;
        transform::Rigid3d pose_to_cb;

        if (matching_result != nullptr) {
            if (matching_result->insertion_result != nullptr) {
                const auto node =
                    matching_result->insertion_result->node_constant_data;
                const auto submaps =
                    matching_result->insertion_result->insertion_submaps;
                auto node_id =
                    pose_graph_->AddNode(node, trajectory_id_, submaps);
                // callback for updated pose and pc
                pose_to_cb = pose_graph_->pose_graph_data()
                                 .trajectory_nodes.at(node_id)
                                 .global_pose;
                auto pose_in_local = matching_result->local_pose;
                auto local_to_global = pose_to_cb * pose_in_local.inverse();
                range_data_callback = sensor::TransformRangeData(
                    matching_result->range_data_in_local,
                    local_to_global.cast<float>());
                range_data_callback.returns.time() = matching_result->time;

            } else {
                pose_to_cb = matching_result->local_pose;
                auto global_pose =
                    pose_graph_->LocalToGlobalTransform(trajectory_id_) *
                    pose_to_cb;
                range_data_callback = sensor::TransformRangeData(
                    matching_result->range_data_in_local,
                    pose_graph_->LocalToGlobalTransform(trajectory_id_)
                        .cast<float>());
                pose_to_cb = global_pose;
            }
            range_data_callback.returns.time() = matching_result->time;
            if (pose_pc_callback_) {
                pose_pc_callback_(pose_to_cb, range_data_callback);
            }
        }
    }
}

void Localization::AddImuData(const sensor::ImuData &imu_data) {
    if (local_slam_ == nullptr)
        return;
    local_slam_->AddImuData(imu_data);
}

void Localization::AddOdometryData(const sensor::OdometryData &odom_data) {
    if (local_slam_ == nullptr)
        return;
    local_slam_->AddOdometryData(odom_data);
}

void Localization::GetInitialPose(
    const std::shared_ptr<RelocPose> &reloc_pose) {
    reloc_pose_ = reloc_pose;
    state_ = State::RELOCATION_SUCESS;
}

void Localization::GetRelocPose(const mapping::NodeId &id,
                                const RelocPose &pose) {
    // Get the vision reloc constraint when location normally
    LOG(INFO) << "still not implement";
}

bool Localization::InitialPoseGraph() {
    // check the reloc pose is nullptr?
    const auto nodes = map_loader_->nodes();
    const auto submaps = map_loader_->submaps();
    const auto constraints = map_loader_->constraints();
    int trajectory_id = map_loader_->trajectory_id();
    pose_graph_->RecoverPoseGraphFromLast(trajectory_id, nodes, submaps,
                                          constraints);
    trajectory_id_ = trajectory_id + 1;
    LOG(INFO) << "pose graph global submap pose of trajectory 0 is: "
              << pose_graph_->pose_graph_data()
                     .global_submap_poses_2d.SizeOfTrajectoryOrZero(
                         trajectory_id);
    return true;
}

} // namespace laser_slam
} // namespace cartographer