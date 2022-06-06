/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <iomanip>
#include "pose_graph/constraint_builder.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
ConstraintBuilder::ConstraintBuilder(
    const ConstraintBuilderParam& param,
    common::ThreadPoolInterface* const thread_pool)
    : param_(param),
      thread_pool_(thread_pool),
      when_done_(nullptr),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(param.ceres_param) {}

ConstraintBuilder::~ConstraintBuilder() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder::FindRelocLoopConstraint(
    const mapping::SubmapId& submap_id, const mapping::Submap2D* const submap,
    const mapping::NodeId& node_id,
    const optimization::TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      param_.max_reloc_constraint_distance) {
    return;
  }
  LOG(INFO) << "find a submap" << submap_id.submap_index;
  while (when_done_) {
    usleep(1000);
    // LOG(WARNING) << "Find Reloc Loop was called while WhenDone was schedule";
  }
  constraints_.emplace_back();
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstrution(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, constant_data,
                      initial_relative_pose, *scan_matcher, true, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder::AddLocalLoopConstraint(
    const mapping::SubmapId& submap_id, const mapping::Submap2D* const submap,
    const mapping::NodeId& node_id,
    const optimization::TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      param_.max_constraint_distance) {
    LOG(INFO) << "far away";
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(param_.ratio))
           .first->second.Pulse()) {
    LOG(INFO) << "no pulse";
    return;
  } else {
    LOG(INFO) << "pulse";
  }
  if (when_done_) {
    LOG(WARNING)
        << "Local Loop Closure was callled while WhenDone was scheduled";
  }
  constraints_.emplace_back();
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstrution(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, constant_data,
                      initial_relative_pose, *scan_matcher, false, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder::ComputeConstraint(
    const mapping::SubmapId& submap_id, const mapping::Submap2D* submap,
    const mapping::NodeId& node_id,
    const optimization::TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher, bool match_full_submap,
    std::unique_ptr<optimization::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d init_pose =
      transform::Project2D(submap->local_pose()) * initial_relative_pose;
  float score = 0.f;
  transform::Rigid2d pose_estimated;
  bool success = false;
  if (match_full_submap) {
    success =
        submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            param_.reloc_min_score, &score, &pose_estimated);
  } else {
    success = submap_scan_matcher.fast_correlative_scan_matcher->Match(
        init_pose, constant_data->filtered_gravity_aligned_point_cloud,
        param_.min_score, &score, &pose_estimated);
  }
  if (not success) {
    return;
  }
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_estimated.translation(), pose_estimated,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimated,
                            &summary);
  const transform::Rigid2d constraint_transform =
      transform::Project2D(submap->local_pose()).inverse() * pose_estimated;
  constraint->reset(
      new optimization::Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    param_.loop_closure_translation_weight,
                                    param_.loop_closure_rotation_weight},
                                   optimization::Constraint::INTER_SUBMAP});
  if (param_.log_matches) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    const transform::Rigid2d difference = init_pose.inverse() * pose_estimated;
    info << " differs by translation " << std::setprecision(2)
         << difference.translation().norm() << " rotation "
         << std::setprecision(3) << std::abs(difference.normalized_angle());
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

const ConstraintBuilder::SubmapScanMatcher*
ConstraintBuilder::DispatchScanMatcherConstrution(
    const mapping::SubmapId& submap_id, const mapping::Grid2D* grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  submap_scan_matcher.grid = grid;
  auto& param = param_.fast_param;
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem([&submap_scan_matcher, &param]() {
    submap_scan_matcher.fast_correlative_scan_matcher =
        absl::make_unique<mapping::scan_matching::FastCorrelativeScanMatcher2D>(
            *submap_scan_matcher.grid, param);
  });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder::NotifyEndOfOneRelocNode() {
  absl::MutexLock locker(&reloc_mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&reloc_mutex_);
    ++num_reloc_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
}

void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

void ConstraintBuilder::RelocWhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  Result result;
  for (const std::unique_ptr<optimization::Constraint>& constraint :
       constraints_) {
    if (constraint == nullptr) continue;
    result.push_back(*constraint);
  }
  if (param_.log_matches) {
    LOG(INFO) << constraints_.size() << " computations resulted in "
              << result.size() << " additional constraints.";
  }
  constraints_.clear();
  callback(result);
}

int ConstraintBuilder::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

int ConstraintBuilder::GetNumFinishedRelocNodes() {
  absl::MutexLock locker(&reloc_mutex_);
  return num_reloc_nodes_;
}

void ConstraintBuilder::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<optimization::Constraint>& constraint :
         constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (param_.log_matches) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
  }
  (*callback)(result);
  // when the scan matching task time consuming, notify function was not execute
  // yet TO_DO(feixiang) : change the strategy, judge is finished until all task
  // end
}

void ConstraintBuilder::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
}
}  // namespace optimization

}  // namespace pose_graph
}  // namespace cartographer
