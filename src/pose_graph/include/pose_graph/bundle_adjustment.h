/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_BUNDLE_ADJUSTMENT_H_
#define POSE_GRAPH_BUNDLE_ADJUSTMENT_H_
#include <absl/container/flat_hash_map.h>
#include <absl/synchronization/mutex.h>
#include <memory>
#include <set>

#include "common/param.h"
#include "mapping/pose_graph.pb.h"
#include "pose_graph/constraint_builder.h"
#include "pose_graph/data_set/pose_graph_data.h"
#include "pose_graph/data_set/work_queue.h"
#include "pose_graph/optimization_problem/optimization_problem_2d.h"
#include "range_data_matching/map/id.h"
#include "range_data_matching/map/submap_2d.h"
namespace cartographer {
namespace pose_graph {
namespace optimization {
typedef std::function<void(const mapping::SubmapId &,
                           const std::shared_ptr<const mapping::Submap> &)>
    SubmapCallback;
class BundleAdjustment {
  public:
    BundleAdjustment(const PoseGraph2DParam &param,
                     common::ThreadPool *thread_pool)
        : param_(param), optimization_problem_(nullptr),
          thread_pool_(thread_pool),
          constraint_builder_(param.constraint_builder_param, thread_pool),
          work_queue_(nullptr), reloc_work_queue_(nullptr) {
        optimization_problem_ =
            std::make_unique<OptimizationProblem2D>(param.optimization_param);
    }
    virtual ~BundleAdjustment() {
        WaitForAllComputations();
        absl::MutexLock locker(&work_queue_mutex_);
        CHECK(work_queue_ == nullptr);
    }

    bool RecoverPoseGraphFromLast(
        int trajectory_id,
        const mapping::MapById<NodeId, TrajectoryNode> &nodes,
        const mapping::MapById<SubmapId, SubmapData> &submaps,
        const std::vector<Constraint> &constraints);

    // Add Node from local slam matching result
    NodeId AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data,
                   int trajectory_id,
                   const std::vector<std::shared_ptr<const mapping::Submap2D>>
                       &insertion_submaps);

    // Add Relocalization Node from local slam matching result
    NodeId AddLocalizationNode(
        std::shared_ptr<const TrajectoryNode::Data> constant_data,
        int trajectory_id,
        const std::vector<std::shared_ptr<const mapping::Submap2D>>
            &insertion_submaps);

    // Add first reloc node for connect new nodes to old pose graph
    NodeId AddFirstRelocNode(
        int trajectory_id,
        const std::vector<std::shared_ptr<const mapping::Submap2D>>
            &insertion_submaps);

    // find reloc constraint, return true if found
    bool
    FindRelocConstraints(std::shared_ptr<const TrajectoryNode> constant_data,
                         int trajectory_id,
                         transform::Rigid3d *pose_after_optimized);

    // Run Final Optimization when all process done
    void RunFinalOptimization();

    // Get the latest node add to pose graph last time
    TrajectoryNode GetLatestNodeData(int trajectory_id) const
        LOCKS_EXCLUDED(pose_graph_mutex_);

    PoseGraphData pose_graph_data() const LOCKS_EXCLUDED(pose_graph_mutex_);

    // maybe pose graph data from different trajectory id
    protos::mapping::proto::PoseGraph
    ToProto(int trajectory_id, bool include_unfinished_submaps) const;

    void SetSubmapCallback(const SubmapCallback &callback) {
        submap_callback_ = callback;
    }

    transform::Rigid3d LocalToGlobalTransform(int trajectory_id) {
        return GetLocalToGlobalTransform(trajectory_id);
    }

    bool is_working() { return is_working_; }

    bool is_reloc() { return is_reloc_; }

    bool Stop();

  private:
    bool LoopVerify(const optimization::Constraint &constraint);

    void RelocConstraintsWorkHandle(const ConstraintBuilder::Result &result);

    bool ComputeRelocConstraintsForNode(const NodeId &node_id);

    void TrimSubmap(const SubmapId &id);

    WorkItem::Result ComputeConstraintsForNode(
        const NodeId &node_id,
        std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
        bool newly_finished_submap) LOCKS_EXCLUDED(pose_graph_mutex_);

    WorkItem::Result ComputeConstraintsForRelocNode(
        const NodeId &node_id,
        std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
        bool newly_finished_submap) LOCKS_EXCLUDED(pose_graph_mutex_);

    transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
        LOCKS_EXCLUDED(pose_graph_mutex_);
    // actually, global frame is same with local frame, but the difference is
    // global pose will change with time that optimization problem triggered
    transform::Rigid3d ComputeLocalToGlobalTransform(
        const mapping::MapById<SubmapId, SubmapSpec2D> &global_submap_poses,
        int trajectory_id) const EXCLUSIVE_LOCKS_REQUIRED(pose_graph_mutex_);

    NodeId
    AppendNode(std::shared_ptr<const TrajectoryNode::Data> constant_data,
               int trajectory_id,
               const std::vector<std::shared_ptr<const mapping::Submap2D>>
                   &insertion_submaps,
               const transform::Rigid3d &optimized_pose)
        LOCKS_EXCLUDED(pose_graph_mutex_);
    void DrainWorkQueue() LOCKS_EXCLUDED(pose_graph_mutex_)
        LOCKS_EXCLUDED(work_queue_mutex_);

    void DrainRelocWorkQueue() LOCKS_EXCLUDED(pose_graph_mutex_)
        LOCKS_EXCLUDED(reloc_work_queue_mutex_);
    // Add Work Item to thread pool
    void AddWorkItem(const std::function<WorkItem::Result()> &work_item)
        LOCKS_EXCLUDED(pose_graph_mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

    // Add Reloc Work Item to thread pool
    void AddRelocWorkItem(const std::function<WorkItem::Result()> &work_item)
        LOCKS_EXCLUDED(pose_graph_mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

    // Handle Work Queue
    void HandleWorkQueue(const ConstraintBuilder::Result &result)
        LOCKS_EXCLUDED(pose_graph_mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

    // Handle Reloc Work Queue
    void HandleRelocWorkQueue(const ConstraintBuilder::Result &result)
        LOCKS_EXCLUDED(pose_graph_mutex_)
            LOCKS_EXCLUDED(reloc_work_queue_mutex_);

    // Compute Constraints between node and submap
    void ComputeConstraint(const NodeId &node_id, const SubmapId &submap_id)
        LOCKS_EXCLUDED(pose_graph_mutex_);

    // Compute Constraints between reloc node and submap
    void ComputeRelocConstraint(const NodeId &node_id,
                                const SubmapId &submap_id)
        LOCKS_EXCLUDED(pose_graph_mutex_);

    // Run Optimization
    void RunOptimization() LOCKS_EXCLUDED(pose_graph_mutex_);

    std::map<int, TrajectoryState> GetTrajectoryStates() const
        LOCKS_EXCLUDED(pose_graph_mutex_);

    // Initialize Submap pose newly insert into optimization problem
    // every time optimization run, submap global pose and node global pose
    // changed we should update new submap pose by delta pose from last time
    std::vector<SubmapId> InitializeGlobalSubmapPose(
        int trajectory_id, const common::Time &time,
        const std::vector<std::shared_ptr<const mapping::Submap2D>>
            &insertion_submaps) EXCLUSIVE_LOCKS_REQUIRED(pose_graph_mutex_);

    // Waits until all computation have finished
    void WaitForAllComputations() LOCKS_EXCLUDED(pose_graph_mutex_)
        LOCKS_EXCLUDED(work_queue_mutex_);

    void WaitForAllRelocComputations() LOCKS_EXCLUDED(pose_graph_mutex_)
        LOCKS_EXCLUDED(reloc_work_queue_mutex_);

    // Get all submap data under lock
    mapping::MapById<SubmapId, SubmapData> GetSubmapDataUnderLock() const
        EXCLUSIVE_LOCKS_REQUIRED(pose_graph_mutex_);

    // Get submap data by id
    SubmapData GetSubmapDataUnderLock(const SubmapId &submap_id) const
        EXCLUSIVE_LOCKS_REQUIRED(pose_graph_mutex_);

    // Get Trajectory nodes
    mapping::MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const
        EXCLUSIVE_LOCKS_REQUIRED(pose_graph_mutex_);
    int num_nodes_since_last_loop_closure_ = 0;
    bool found_reloc_constraints_ = false;
    bool is_reloc_ = false;
    bool is_working_ = false;
    mutable absl::Mutex pose_graph_mutex_;
    absl::Mutex work_queue_mutex_;
    absl::Mutex reloc_work_queue_mutex_;
    absl::Mutex num_reloc_nodes_mutex_;
    std::mutex num_nodes_since_loop_closure_mutex_;
    int num_reloc_nodes_ = 0 GUARDED_BY(num_reloc_nodes_mutex_);
    PoseGraph2DParam param_;
    std::unique_ptr<OptimizationProblem2D>
        optimization_problem_; // optimization problem main
    common::ThreadPool
        *const thread_pool_; // thread pool for handling work queue
    ConstraintBuilder constraint_builder_; // search constraint
    PoseGraphData data_;                   // all data in pose graph
    std::unique_ptr<WorkQueue> work_queue_ GUARDED_BY(work_queue_mutex_);
    std::unique_ptr<WorkQueue>
        reloc_work_queue_ GUARDED_BY(reloc_work_queue_mutex_);
    SubmapCallback submap_callback_;
    mapping::MapById<NodeId, transform::Rigid3d> vision_id_pose_;
};
typedef std::shared_ptr<BundleAdjustment> BundleAdjustmentPtr;
typedef std::shared_ptr<const BundleAdjustment> BundleAdjustmentConstPtr;
} // namespace optimization
} // namespace pose_graph
} // namespace cartographer

#endif // POSE_GRAPH_BUNDLE_ADJUSTMENT_H_
