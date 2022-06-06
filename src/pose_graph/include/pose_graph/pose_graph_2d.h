/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_POSE_GRAPH_2D_H_
#define POSE_GRAPH_POSE_GRAPH_2D_H_
#include <memory>
#include <mutex>
#include <map>
#include <iomanip>

#include <absl/container/flat_hash_map.h>
#include <absl/synchronization/mutex.h>

#include "common/param.h"
#include "pose_graph/pose_graph_interface.h"
#include "pose_graph/data_set/pose_graph_data.h"
#include "pose_graph/data_set/work_queue.h"
#include "range_data_matching/map/id.h"
#include "range_data_matching/map/submap_2d.h"
#include "pose_graph/optimization_problem/optimization_problem_2d.h"
#include "pose_graph/constraint_builder.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
class PoseGraph2D : public PoseGraph<OdometryDelta2D> {
 public:
  PoseGraph2D(
      PoseGraph2DParam param,
      std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
      common::ThreadPool* thread_pool)
      : param_(param),
        global_loop_closure_signal_(false),
        optimization_problem_(std::move(optimization_problem)),
        thread_pool_(thread_pool),
        constraint_builder_(param.constraint_builder_param, thread_pool) {}

  ~PoseGraph2D() override {
    WaitForAllComputations();
    absl::MutexLock locker(&work_queue_mutex_);
    CHECK(work_queue_ == nullptr);
  }

  void AddOdometryDeltaData(const OdometryDelta2D& delta_odom) override {
    LOG(INFO) << "Still not implemented" << delta_odom.delta_pose;
  }

  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id) override;

  void SetGlobalSlamOptimizationCallback(
      PoseGraph::GlobalSlamOptimizationCallback callback) override;

  void RunFinalOptimization() override;
  // Add new node which is determined by scan matching
  // Node is scan match with the front of insertion submaps, but inserted to
  // both map. if insertion_submaps.front().finished(), it will be the last time
  // insert to this map
  NodeId AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data,
                 int trajectory_id,
                 const std::vector<std::shared_ptr<const mapping::Submap2D>>&
                     insertion_submaps);

  std::map<int, TrajectoryState> GetTrajectoryStates() const override
      LOCKS_EXCLUDED(mutex_);
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
      LOCKS_EXCLUDED(mutex_) override;


 private:
  // Handles a new work item.
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item)
      LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);
  // Adds constraints for a node, and starts scan matching in the background.
  optimization::WorkItem::Result ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
      bool newly_finished_submap) LOCKS_EXCLUDED(mutex_);
  // Appends the new node and submap (if needed) to the internal data
  // structures.
  mapping::NodeId AppendNode(
      std::shared_ptr<const optimization::TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const mapping::Submap2D>>&
          insertion_submaps,
      const transform::Rigid3d& optimized_pose) LOCKS_EXCLUDED(mutex_);

  common::Time GetLatestNodeTime(const NodeId& node_id,
                                 const SubmapId& submap_id) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
      LOCKS_EXCLUDED(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const mapping::Submap2D>>&
          insertion_submaps) EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Runs the optimization, executes the trimmers and processes the work queue.
  void HandleWorkQueue(const ConstraintBuilder::Result& result)
      LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

  // Process pending tasks in the work queue on the calling thread, until the
  // queue is either empty or an optimization is required.
  void DrainWorkQueue() LOCKS_EXCLUDED(mutex_)
      LOCKS_EXCLUDED(work_queue_mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() LOCKS_EXCLUDED(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() LOCKS_EXCLUDED(mutex_)
      LOCKS_EXCLUDED(work_queue_mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const mapping::MapById<SubmapId, optimization::SubmapSpec2D>&
          global_submap_poses,
      int trajectory_id) const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  bool CanAddWorkItemModifying(int trajectory_id)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  
  PoseGraph2DParam param_;
  bool global_loop_closure_signal_;
  // Number of nodes added since last loop closure.
  int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;
  mutable absl::Mutex mutex_;
  absl::Mutex work_queue_mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<WorkQueue> work_queue_ GUARDED_BY(work_queue_mutex_);

  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  std::unique_ptr<OptimizationProblem2D> optimization_problem_;
  PoseGraphData data_;
  std::map<int, common::Time> last_connect_time_;
  // Thread pool used for handling the work queue.
  common::ThreadPool* const thread_pool_;
  ConstraintBuilder constraint_builder_;
};
}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_POSE_GRAPH_2D_H_
