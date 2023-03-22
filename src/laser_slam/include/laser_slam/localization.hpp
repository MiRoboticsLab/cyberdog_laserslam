// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 1. Load Map From pbstream
// 2. Generate a new pose graph
// 3. Load the submaps, nodes, constraints as frozen state to new pose graph
// 4. Local Slam start whenever reloc info comming
// 5. reloc info comming, delta pose from now to last time multiply the pose
// last time in local slam to find the loop
// 6. loop detect sucess, every n nodes we find the loop between now and last
// map

// as for the pose graph,
// 1. recover pose graph from pbstream loading
// 2. every node, submap, constraint should be setted frozen(constant)
#ifndef LASER_SLAM__LOCALIZATION_HPP_
#define LASER_SLAM__LOCALIZATION_HPP_
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>

#include "cyberdog_visions_interfaces/srv/reloc.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

#include "laser_slam/local_slam.hpp"
#include "laser_slam/map_loader.hpp"
#include "pose_graph/bundle_adjustment.h"
//  using namespace std::chrono_literals;
namespace cartographer
{
namespace laser_slam
{
struct RelocPose
{
  common::Time time;
  transform::Rigid3d pose;
};

enum class State
{
  INITIALIZATION,
  RELOCATION,
  RELOCATING,
  RELOCATION_SUCESS,
  LOCATION
};

struct LoopJob
{
  enum Type { INIT_LOCATION, NORMAL_LOCATION } type;
  mapping::NodeId node_id;
  LoopJob()
  : type(Type::INIT_LOCATION), node_id(0, 0) {}
  explicit LoopJob(const mapping::NodeId & node_id)
  : type(Type::NORMAL_LOCATION), node_id(node_id) {}
};

typedef std::function<void (const transform::Rigid3d &,
    const sensor::RangeData &)>
  PosePcCallback;
typedef std::deque<LoopJob> JobQueue;

class Localization
{
public:
  explicit Localization(const LocalizationParam & param);
  virtual ~Localization();

  bool Initialize();

  void AddRangeData(const sensor::PointCloud & timed_point_cloud);

  void AddImuData(const sensor::ImuData & imu_data);

  void AddOdometryData(const sensor::OdometryData & odom_data);

  bool Stop();

  void SetCallback(const PosePcCallback & callback);

  void GetInitialPose(const std::shared_ptr<RelocPose> & reloc_pose);

  void GetRelocPose(const mapping::NodeId & id, const RelocPose & pose);

  void SetRelocPublisher(
    const rclcpp_lifecycle::LifecyclePublisher<
      std_msgs::msg::Int32>::SharedPtr & publisher);

private:
  bool InitialPoseGraph();

  void RelocPublisherThread();

  int trajectory_id_;
  int reloc_id_;
  bool is_reloc_;
  bool start_;
  bool inertial_comming_ = false;
  State state_;
  std::mutex reloc_pose_mutex_;
  std::mutex is_reloc_mutex_;
  LocalizationParam param_;
  std::shared_ptr<RelocPose> reloc_pose_;
  LocalSlamPtr local_slam_;
  common::ThreadPool thread_pool_;
  std::shared_ptr<std::thread> reloc_publisher_thread_;
  pose_graph::optimization::BundleAdjustmentPtr pose_graph_;
  MapLoaderPtr map_loader_;
  PosePcCallback pose_pc_callback_;    // For visualize and pub tf
  transform::Rigid3d first_reloc_pose_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
    reloc_publisher_;
};
typedef std::shared_ptr<Localization> LocalizationPtr;
typedef std::shared_ptr<const Localization> LocalizationConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM__LOCALIZATION_HPP_
