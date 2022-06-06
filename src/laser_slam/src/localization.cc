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
Localization::Localization(const LocalizationParam& param)
    : trajectory_id_(0),
      need_vision_reloc_(true),
      is_reloc_(false),
      start_(true),
      reloc_thread_(nullptr),
      param_(param),
      // reloc_client_(nullptr),
      reloc_pose_(nullptr),
      local_slam_(nullptr),
      thread_pool_(param.thread_num_pool),
      pose_graph_(nullptr),
      map_loader_(nullptr) {}
Localization::~Localization() {
  auto constraints = pose_graph_->pose_graph_data();
  LOG(INFO) << constraints.global_submap_poses_2d.SizeOfTrajectoryOrZero(0)
            << " , "
            << constraints.global_submap_poses_2d.SizeOfTrajectoryOrZero(1);
  int count = 0;
  for (auto constraint : constraints.constraints) {
    if (constraint.node_id.trajectory_id !=
        constraint.submap_id.trajectory_id) {
      ++count;
    }
  }
  LOG(INFO) << "Final get : " << count << " parent to son constraint";
}

bool Localization::Stop() {
  start_ = false;
  if (reloc_thread_ && reloc_thread_->joinable()) {
    reloc_thread_->join();
  }
  return true;
}

void Localization::SetCallback(const PosePcCallback& callback) {
  pose_pc_callback_ = callback;
}

bool Localization::Initialize() {
  // reloc_client_.reset(new stream::Client(grpc::CreateChannel(
  //     param_.channel_name, grpc::InsecureChannelCredentials())));

  pose_graph_.reset(new pose_graph::optimization::BundleAdjustment(
      param_.pose_graph_param, &thread_pool_));
  LOG(INFO) << "file path is: " << param_.pbstream_file_path;
  stream::ProtoStreamReader reader(param_.pbstream_file_path);
  stream::ProtoStreamDeserializer deserializer(&reader);
  map_loader_ = std::make_shared<MapLoader>(deserializer);
  InitialPoseGraph();
  LOG(INFO) << "initial pose graph end";

  if (not reloc_thread_)
    reloc_thread_.reset(
        new std::thread(std::bind(&Localization::RelocPoseRequest, this)));
  else
    LOG(WARNING) << "reloc thread already exist";

  LOG(INFO) << "Initialize done";
  return true;
}

void Localization::AddRangeData(const sensor::PointCloud& timed_point_cloud) {
  transform::Rigid3d optimized_pose;
  std::unique_ptr<MatchingResult> matching_result;

  // Initialize Reloc Phase
  // To_DO(feixiang Zeng): Find out reason why is_reloc_ change from true to
  // false when the next data comming
  {
    std::lock_guard<std::mutex> lk1(reloc_pose_mutex_);
    std::lock_guard<std::mutex> lk2(is_reloc_mutex_);
    if (reloc_pose_ == nullptr) {
      LOG(INFO) << "Still no Reloc Pose Recieved";
      return;
    }
    LOG(INFO) << "add range data";
    // if (!is_reloc_)
    if (local_slam_ == nullptr) {
      LOG(INFO) << "start reloc" << is_reloc_;
      // to_do: locker for pose
      transform::Rigid3d pose = reloc_pose_->pose;
      // pose = transform::Rigid3d::Translation(pose.translation());
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
                param_.local_slam_param.missing_data_ray_length / range * delta;
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
      const sensor::PointCloud& filtered_point_cloud =
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
      if (!is_reloc_) {
        LOG(WARNING) << "Reloc falied ";
        return;
      } else {
        LOG(INFO) << "Reloc success, Connect new node to old pose graph";
        first_reloc_pose_ = optimized_pose;
        local_slam_.reset(new LocalSlam(param_.local_slam_param));
        std::vector<transform::TimedRigid3d> initial_poses;
        // Initialize phase, the dog do not run anywhere, the pose from last
        // two or three seconds could view as pose now
        initial_poses.push_back(
            transform::TimedRigid3d{optimized_pose, timed_point_cloud.time()});
        local_slam_->InitialExtrapolatorWithPoses(initial_poses);
        matching_result = local_slam_->AddRangeData(timed_point_cloud);
        if (matching_result != nullptr) {
          if (matching_result->insertion_result != nullptr) {
            const auto submaps =
                matching_result->insertion_result->insertion_submaps;
            pose_graph_->AddFirstRelocNode(trajectory_id_, submaps);
          }
        }
        return;
      }
    }
  }

  matching_result = local_slam_->AddRangeData(timed_point_cloud);
  if (matching_result != nullptr) {
    if (matching_result->insertion_result != nullptr) {
      const auto node = matching_result->insertion_result->node_constant_data;
      const auto submaps = matching_result->insertion_result->insertion_submaps;
      auto node_id = pose_graph_->AddNode(node, trajectory_id_, submaps);
      if (pose_pc_callback_) {
        // callback for updated pose and pc
        auto pose = pose_graph_->pose_graph_data()
                        .trajectory_nodes.at(node_id)
                        .global_pose;
        auto pose_in_local = matching_result->local_pose;
        auto local_to_global = pose * pose_in_local.inverse();
        auto range_data_in_global =
            sensor::TransformRangeData(matching_result->range_data_in_local,
                                       local_to_global.cast<float>());
        pose_pc_callback_(pose, range_data_in_global);
      }
    }
  }
}

void Localization::AddImuData(const sensor::ImuData& imu_data) {
  if (local_slam_ == nullptr) return;
  local_slam_->AddImuData(imu_data);
}

void Localization::AddOdometryData(const sensor::OdometryData& odom_data) {
  if (local_slam_ == nullptr) return;
  local_slam_->AddOdometryData(odom_data);
}

void Localization::RelocPoseRequest() {
  // Send reloc request to server, expect get a reply
  while (start_) {
    if (need_vision_reloc_) {
      Eigen::Quaterniond bearing;
      Eigen::Vector3d position;
      common::Time timestamp;
      RelocPose pose;
      pose.pose = transform::Rigid3d::Identity();
      pose.time = common::Time::max();
      reloc_pose_ = std::make_shared<RelocPose>(pose);
      need_vision_reloc_ = false;
      // std::string user("reloc");
      // auto status =
      //     reloc_client_->GetRelocPose(user, &timestamp, &bearing, &position);
      // if (status.ok()) {
      //   need_vision_reloc_ = false;
      //   pose.time = timestamp;
      //   pose.pose = transform::Rigid3d(position, bearing);
      //   {
      //     std::lock_guard<std::mutex> lk(reloc_pose_mutex_);
      //     reloc_pose_ = std::make_shared<RelocPose>(pose);
      //   }
      //   LOG(INFO) << "Got a Reloc Pose, Exit reloc Thread"
      //             << " Pose is: " << pose.pose.DebugString();
      // } else {
      //   usleep(1000);
      // }
    } else {
      usleep(1000);
    }
  }
}

bool Localization::InitialPoseGraph() {
  // check the reloc pose is nullptr?
  const auto nodes = map_loader_->nodes();
  const auto submaps = map_loader_->submaps();
  const auto constraints = map_loader_->constraints();
  int trajectory_id = map_loader_->trajectory_id();
  LOG(INFO) << "start recover";
  pose_graph_->RecoverPoseGraphFromLast(trajectory_id, nodes, submaps,
                                        constraints);
  LOG(INFO) << "recover end";
  trajectory_id_ = trajectory_id + 1;
  LOG(INFO) << "pose graph global submap pose of trajectory 0 is: "
            << pose_graph_->pose_graph_data()
                   .global_submap_poses_2d.SizeOfTrajectoryOrZero(
                       trajectory_id);
  return true;
}

}  // namespace laser_slam
}  // namespace cartographer