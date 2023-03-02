/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define POSE_GRAPH_CONSTRAINT_BUILDER_H_
#include <deque>
#include <functional>
#include <map>

#include <absl/synchronization/mutex.h>

#include "common/fixed_ratio_sampler.h"
#include "common/param.h"
#include "common/task.h"
#include "common/thread_pool.h"
#include "pose_graph/data_set/pose_graph_data.h"
#include "pose_graph/data_set/trajectory_node.h"
#include "range_data_matching/map/submap_2d.h"
#include "range_data_matching/scan_matching/ceres_scan_matcher_2d.h"
#include "range_data_matching/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "sensor/point_cloud.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
class ConstraintBuilder {
  public:
    using Result = std::vector<optimization::Constraint>;
    ConstraintBuilder(const ConstraintBuilderParam &param,
                      common::ThreadPoolInterface *thread_pool);
    virtual ~ConstraintBuilder();

    ConstraintBuilder(const ConstraintBuilder &) = delete;
    ConstraintBuilder &operator=(const ConstraintBuilder &) = delete;

    void SetLocalizationMode(bool localization) {
        localization_mode_ = localization;
    }

    void SetIsReloc() { is_reloc_ = true; }

    void AddLocalLoopConstraint(
        const mapping::SubmapId &submap_id,
        const mapping::Submap2D *const submap, const mapping::NodeId &node_id,
        const optimization::TrajectoryNode::Data *const constant_data,
        const transform::Rigid2d &initial_relative_pose);

    // Find reloc constraint when localization mode
    void FindRelocLoopConstraint(
        const mapping::SubmapId &submap_id,
        const mapping::Submap2D *const submap, const mapping::NodeId &node_id,
        const optimization::TrajectoryNode::Data *const constant_data,
        const transform::Rigid2d &initial_relative_pose);

    // Must be called after all computations related to one node have been
    // added.
    void NotifyEndOfNode();

    // Run When reloc computations, Must called after all computations related
    // to one node have been added
    void NotifyEndOfOneRelocNode();

    // Registers the 'callback' to be called with the results, after all
    // computations triggered by 'MaybeAdd*Constraint' have finished.
    // 'callback' is executed in the 'ThreadPool'.
    void WhenDone(const std::function<void(const Result &)> &callback);

    // Returns the number of consecutive finished nodes.
    int GetNumFinishedNodes();

    // Retruns the number of finished reloc nodes
    int GetNumFinishedRelocNodes();

    // Delete data related to 'submap_id'.
    void DeleteScanMatcher(const SubmapId &submap_id);

    // When reloc job done, call back data
    void RelocWhenDone(const std::function<void(const Result &)> &callback);

  private:
    struct SubmapScanMatcher {
        const mapping::Grid2D *grid = nullptr;
        std::unique_ptr<mapping::scan_matching::FastCorrelativeScanMatcher2D>
            fast_correlative_scan_matcher;
        std::weak_ptr<common::Task> creation_task_handle;
    };

    const SubmapScanMatcher *
    DispatchScanMatcherConstrution(const mapping::SubmapId &submap_id,
                                   const mapping::Grid2D *grid)
        EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // compute constraint for two node in graph
    void ComputeConstraint(
        const mapping::SubmapId &submap_id, const mapping::Submap2D *submap,
        const mapping::NodeId &node_id,
        const optimization::TrajectoryNode::Data *const constant_data,
        const transform::Rigid2d &initial_relative_pose,
        const SubmapScanMatcher &submap_scan_matcher, bool match_full_submap,
        std::unique_ptr<optimization::Constraint> *constaint)
        LOCKS_EXCLUDED(mutex_);

    void RunWhenDoneCallback() LOCKS_EXCLUDED(mutex_);

    void RunRelocWhenDoneCallback() LOCKS_EXCLUDED(reloc_mutex_);

    const ConstraintBuilderParam param_;
    common::ThreadPoolInterface *thread_pool_;
    absl::Mutex mutex_;
    absl::Mutex reloc_mutex_;

    // 'callback' set by WhenDone().
    std::unique_ptr<std::function<void(const Result &)>>
        when_done_ GUARDED_BY(mutex_);

    std::unique_ptr<std::function<void(const Result &)>>
        reloc_when_done_ GUARDED_BY(reloc_mutex_);

    int num_started_nodes_ GUARDED_BY(mutex_) = 0;

    int num_finished_nodes_ GUARDED_BY(mutex_) = 0;

    int num_reloc_started_nodes_ GUARDED_BY(reloc_mutex_) = 0;

    int num_reloc_nodes_ GUARDED_BY(reloc_mutex_) = 0;

    bool localization_mode_ = false;

    bool is_reloc_ = false;

    std::unique_ptr<common::Task> finish_node_task_ GUARDED_BY(mutex_);

    std::unique_ptr<common::Task> when_done_task_ GUARDED_BY(mutex_);

    std::unique_ptr<common::Task>
        finish_reloc_node_task_ GUARDED_BY(reloc_mutex_);

    std::unique_ptr<common::Task> reloc_when_done_task_ GUARDED_BY(mutex_);

    // Constraints currently being computed in the background. A deque is used
    // to keep pointers valid when adding more entries. Constraint search
    // results with below-threshold scores are also 'nullptr'.
    std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

    std::deque<std::unique_ptr<Constraint>>
        reloc_constraints_ GUARDED_BY(reloc_mutex_);

    std::map<mapping::SubmapId, SubmapScanMatcher>
        submap_scan_matchers_ GUARDED_BY(mutex_) GUARDED_BY(reloc_mutex_);
    std::map<mapping::SubmapId, common::FixedRatioSampler> per_submap_sampler_;

    mapping::scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
};
} // namespace optimization
} // namespace pose_graph
} // namespace cartographer

#endif // POSE_GRAPH_CONSTRAINT_BUILDER_H_
