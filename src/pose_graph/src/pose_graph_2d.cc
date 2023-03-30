/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "pose_graph/pose_graph_2d.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>&
        insertion_submaps) {
  // append node to pose graph data
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
  LOG(INFO) << "optimized pose is: " << optimized_pose.DebugString();
  const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                    insertion_submaps, optimized_pose);
  LOG(INFO) << "node appended";
  // compute constraint for nodes and return the status which determine if run
  // optimization
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
  return node_id;
}

void PoseGraph2D::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) {
  absl::MutexLock locker(&work_queue_mutex_);
  if (work_queue_ == nullptr) {
    work_queue_ = absl::make_unique<WorkQueue>();
    auto task = absl::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }
  const auto now = std::chrono::steady_clock::now();
  LOG(INFO) << "work queue push back";
  work_queue_->push_back({now, work_item});
}

bool PoseGraph2D::CanAddWorkItemModifying(int trajectory_id) {
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    return true;
  }
  if (it->second.state == TrajectoryState::FINISHED) {
    // TODO(gaschler): Replace all FATAL to WARNING after some testing.
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has finished "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.deletion_state !=
      InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been scheduled for deletion "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.state == TrajectoryState::DELETED) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  return true;
}

mapping::NodeId PoseGraph2D::AppendNode(
    std::shared_ptr<const optimization::TrajectoryNode::Data> constant_data,
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>&
        insertion_submaps,
    const transform::Rigid3d& optimized_pose) {
  absl::MutexLock locker(&mutex_);
  if (!CanAddWorkItemModifying(trajectory_id)) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }
  LOG(INFO) << "3";
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id,
      optimization::TrajectoryNode{constant_data, optimized_pose});
  LOG(INFO) << "Node id appended";
  ++data_.num_trajectory_nodes;
  // End of Trajectory return the iterator which point to the begin of next
  // trajectory
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()) {
    const mapping::SubmapId submap_id = data_.submap_data.Append(
        trajectory_id, optimization::InternalSubmapData());
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
  }
  LOG(INFO) << "2";
  return node_id;
}

std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
    int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>&
        insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_->submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    LOG(INFO) << "Size of trajectory or zero is: "
              << submap_data.SizeOfTrajectoryOrZero(trajectory_id);
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      optimization_problem_->AddSubMap(
          trajectory_id, transform::Project2D(
                             ComputeLocalToGlobalTransform(
                                 data_.global_submap_poses_2d, trajectory_id) *
                             insertion_submaps[0]->local_pose()));
    }
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const SubmapId submap_id{trajectory_id, 0};
    CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const SubmapId last_submap_id = std::prev(end_it)->id;
  if (data_.submap_data.at(last_submap_id).submap ==
      insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    optimization_problem_->AddSubMap(
        trajectory_id,
        first_submap_pose *
            transform::Project2D(insertion_submaps[0]->local_pose().inverse() *
                                 insertion_submaps[1]->local_pose()));
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

void PoseGraph2D::RunFinalOptimization() {
  {
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          param_.max_num_final_iterations);
      return WorkItem::Result::kRunOptimization;
    });
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          param_.optimization_param.max_num_iterations);
      return WorkItem::Result::kDoNotRunOptimization;
    });
  }
  WaitForAllComputations();
}

void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  AddWorkItem([this, node_id, submap_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::WaitForAllComputations() {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(1.)))) {
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  absl::MutexLock locker(&mutex_);
  bool notification = false;
  constraint_builder_.WhenDone(
      [this, &notification](const ConstraintBuilder::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });
  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };
  while (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(1.)))) {
    report_progress();
  }
  CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

optimization::WorkItem::Result PoseGraph2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
    bool newly_finished_submap) {
  std::vector<SubmapId> submap_ids;
  std::vector<SubmapId> finished_submap_ids;
  std::set<NodeId> newly_finished_submap_node_ids;
  {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    submap_ids = InitializeGlobalSubmapPoses(
        node_id.trajectory_id, constant_data->time, insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());
    const SubmapId matching_id = submap_ids.front();
    const transform::Rigid2d local_pose_2d =
        transform::Project2D(constant_data->local_pose *
                             transform::Rigid3d::Rotation(
                                 constant_data->gravity_alignment.inverse()));
    // global pose 2d which translation only
    const transform::Rigid2d global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        transform::Project2D(insertion_submaps.front()->local_pose())
            .inverse() *
        local_pose_2d;
    LOG(INFO) << "add trajectory node";
    optimization_problem_->AddTrajectoryNode(
        matching_id.trajectory_id,
        optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                                 global_pose_2d,
                                 constant_data->gravity_alignment});
    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
      const SubmapId submap_id = submap_ids[i];
      CHECK(data_.submap_data.at(submap_id).state ==
            SubmapState::kNoConstraintSearch);
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);
      const transform::Rigid2d constraint_transform =
          transform::Project2D(insertion_submaps[i]->local_pose()).inverse() *
          local_pose_2d;
      data_.constraints.push_back(Constraint{
          submap_id,
          node_id,
          {transform::Embed3D(constraint_transform),
           param_.matcher_translation_weight, param_.matcher_rotation_weight},
          Constraint::INTRA_SUBMAP});
    }
    for (const auto& submap_id_data : data_.submap_data) {
      if (submap_id_data.data.state == SubmapState::kFinished) {
        CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
        finished_submap_ids.emplace_back(submap_id_data.id);
      }
    }
    if (newly_finished_submap) {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      InternalSubmapData& finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      finished_submap_data.state = SubmapState::kFinished;
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  }
  LOG(INFO) << "1";
  for (const auto& submap_id : finished_submap_ids) {
    ComputeConstraint(node_id, submap_id);
  }
  if (newly_finished_submap) {
    const SubmapId newly_finished_submap_id = submap_ids.front();
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    for (const auto& node_id_data : optimization_problem_->node_data()) {
      const NodeId& node_id = node_id_data.id;
      if (newly_finished_submap_node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, newly_finished_submap_id);
      }
    }
  }
  constraint_builder_.NotifyEndOfNode();
  absl::MutexLock locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;
  LOG(INFO) << "num nodes since last loop closure"
            << num_nodes_since_last_loop_closure_;
  // Optimization triger conditionally by node accumulated
  if (param_.optimize_every_n_nodes > 0 &&
      num_nodes_since_last_loop_closure_ > param_.optimize_every_n_nodes) {
    return WorkItem::Result::kRunOptimization;
  }
  return WorkItem::Result::kDoNotRunOptimization;
}

transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  absl::MutexLock locker(&mutex_);
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

void PoseGraph2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  bool add_local_loop = false;
  const optimization::TrajectoryNode::Data* constant_data;
  const mapping::Submap2D* submap;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.submap_data.at(submap_id).state ==
          optimization::SubmapState::kFinished);
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
      // only finished submap search constraint
      return;
    }

    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    submap = static_cast<const mapping::Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
    // const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
  }
  // if (not global_loop_closure_signal_) {
  const transform::Rigid2d initial_relative_pose =
      optimization_problem_->submap_data().at(submap_id).global_pose.inverse() *
      optimization_problem_->node_data().at(node_id).global_pose_2d;
  constraint_builder_.AddLocalLoopConstraint(
      submap_id, submap, node_id, constant_data, initial_relative_pose);
  //}
}

transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)
        ->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(data_.trajectory_nodes.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  return transform::Interpolate(
             transform::TimedRigid3d{std::prev(it)->data.global_pose,
                                     std::prev(it)->data.time()},
             transform::TimedRigid3d{it->data.global_pose, it->data.time()},
             time)
      .pose;
}

transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const mapping::MapById<SubmapId, optimization::SubmapSpec2D>&
        global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it != data_.initial_trajectory_poses.end()) {
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
  }
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

void PoseGraph2D::SetGlobalSlamOptimizationCallback(
    PoseGraph::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

void PoseGraph2D::HandleWorkQueue(const ConstraintBuilder::Result& result) {
  {
    absl::MutexLock locker(&mutex_);
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }
  RunOptimization();

  if (global_slam_optimization_callback_) {
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      absl::MutexLock locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      for (const int trajectory_id : node_data.trajectory_ids()) {
        if (node_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
          continue;
        }
        trajectory_id_to_last_optimized_node_id.emplace(
            trajectory_id,
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id);
        trajectory_id_to_last_optimized_submap_id.emplace(
            trajectory_id,
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id);
      }
    }
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }
  num_nodes_since_last_loop_closure_ = 0;

  DrainWorkQueue();
}

void PoseGraph2D::DrainWorkQueue() {
  bool process_work_queue = true;
  size_t work_queue_size;
  while (process_work_queue) {
    std::function<WorkItem::Result()> work_item;
    {
      absl::MutexLock locker(&work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      LOG(INFO) << "pop front";
      work_queue_size = work_queue_->size();
    }
    process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
    if (process_work_queue) {
      LOG(INFO) << "no optimization";
    }
  }
  LOG(INFO) << "Remaining work items in queue: " << work_queue_size;
  // We have to optimize again.
  constraint_builder_.WhenDone([this](const ConstraintBuilder::Result& result) {
    HandleWorkQueue(result);
  });
}

std::map<int, TrajectoryState> PoseGraph2D::GetTrajectoryStates() const {
  std::map<int, TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

void PoseGraph2D::RunOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_,
  // data_.constraints, data_.frozen_trajectories and data_.landmark_nodes
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates());
  LOG(INFO) << "Optimization Problem Triggered";
  absl::MutexLock locker(&mutex_);

  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();
  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.global_pose_2d) *
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        data_.global_submap_poses_2d, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it =
        std::next(data_.trajectory_nodes.find(last_optimized_node_id));
    for (; node_it != data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  data_.global_submap_poses_2d = submap_data;
}

}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer
