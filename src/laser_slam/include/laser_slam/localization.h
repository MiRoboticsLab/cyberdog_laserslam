/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

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
#ifndef LASER_SLAM_LOCALIZATION_H_
#define LASER_SLAM_LOCALIZATION_H_
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "cyberdog_visions_interfaces/srv/reloc.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

#include "laser_slam/map_loader.h"
#include "laser_slam/local_slam.h"
#include "pose_graph/bundle_adjustment.h"
using namespace std::chrono_literals;
namespace cartographer {
namespace laser_slam {
struct RelocPose {
  common::Time time;
  transform::Rigid3d pose;
};

enum class State {
  INITIALIZATION,
  RELOCATION,
  RELOCATING,
  RELOCATION_SUCESS,
  LOCATION
};

struct LoopJob {
  enum Type { INIT_LOCATION, NORMAL_LOCATION } type;
  mapping::NodeId node_id;
  LoopJob() : type(Type::INIT_LOCATION), node_id(0, 0) {}
  explicit LoopJob(const mapping::NodeId& node_id)
      : type(Type::NORMAL_LOCATION), node_id(node_id) {}
};

typedef std::function<void(const transform::Rigid3d&, const sensor::RangeData&)>
    PosePcCallback;
typedef std::deque<LoopJob> JobQueue;

class Localization {
 public:
  explicit Localization(const LocalizationParam& param);
  virtual ~Localization();

  bool Initialize();

  void AddRangeData(const sensor::PointCloud& timed_point_cloud);

  void AddImuData(const sensor::ImuData& imu_data);

  void AddOdometryData(const sensor::OdometryData& odom_data);

  bool Stop();

  void SetCallback(const PosePcCallback& callback);

  void GetInitialPose(const std::shared_ptr<RelocPose>& reloc_pose);

  void GetRelocPose(const mapping::NodeId& id, const RelocPose& pose);

  void SetRelocPublisher(const rclcpp_lifecycle::LifecyclePublisher<
                         std_msgs::msg::Int32>::SharedPtr& publisher);

  State state() { return state_; }

 private:
  bool InitialPoseGraph();

  int trajectory_id_;
  int reloc_id_;
  bool is_reloc_;
  bool start_;
  State state_;
  std::mutex reloc_pose_mutex_;
  std::mutex is_reloc_mutex_;
  LocalizationParam param_;
  std::shared_ptr<RelocPose> reloc_pose_;
  LocalSlamPtr local_slam_;
  common::ThreadPool thread_pool_;
  pose_graph::optimization::BundleAdjustmentPtr pose_graph_;
  MapLoaderPtr map_loader_;
  PosePcCallback pose_pc_callback_;  // For visualize and pub tf
  transform::Rigid3d first_reloc_pose_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr
      reloc_publisher_;
};
typedef std::shared_ptr<Localization> LocalizationPtr;
typedef std::shared_ptr<const Localization> LocalizationConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_LOCALIZATION_H_
