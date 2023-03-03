/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "pose_graph/bundle_adjustment.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
namespace {
protos::mapping::proto::PoseGraph::Constraint::Tag
ToProto(const Constraint::Tag &tag) {
    switch (tag) {
    case Constraint::Tag::INTRA_SUBMAP:
        return protos::mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP;
    case Constraint::Tag::INTER_SUBMAP:
        return protos::mapping::proto::PoseGraph::Constraint::INTER_SUBMAP;
    }
    LOG(FATAL) << "Unsupported tag.";
}
protos::mapping::proto::PoseGraph::Constraint
ToProto(const Constraint &constraint) {
    protos::mapping::proto::PoseGraph::Constraint constraint_proto;
    constraint_proto.set_translation_weight(constraint.pose.translation_weight);
    constraint_proto.set_rotation_weight(constraint.pose.rotation_weight);
    *constraint_proto.mutable_relative_pose() =
        transform::ToProto(constraint.pose.zbar_ij);
    constraint_proto.mutable_submap_id()->set_submap_index(
        constraint.submap_id.submap_index);
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        constraint.submap_id.trajectory_id);
    constraint_proto.mutable_node_id()->set_node_index(
        constraint.node_id.node_index);
    constraint_proto.mutable_node_id()->set_trajectory_id(
        constraint.node_id.trajectory_id);
    constraint_proto.set_tag(ToProto(constraint.tag));
    return constraint_proto;
}
} // namespace
NodeId BundleAdjustment::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>
        &insertion_submaps) {
    // append node which update pose up to last optimized node
    const transform::Rigid3d optimized_pose(
        GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
    LOG_EVERY_N(INFO, 100) << "newest pose is: "
                           << optimized_pose.DebugString();
    const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                      insertion_submaps, optimized_pose);
    // if the front of submaps finished?
    const bool newly_finished_submap =
        insertion_submaps.front()->insertion_finished();
    // search constraint locally
    AddWorkItem([=]() LOCKS_EXCLUDED(pose_graph_mutex_) {
        return ComputeConstraintsForNode(node_id, insertion_submaps,
                                         newly_finished_submap);
    });
    return node_id;
}

bool BundleAdjustment::Stop() {
    constraint_builder_.SetIsReloc();
    constraint_builder_.SetLocalizationMode(true);
    if (reloc_work_queue_ != nullptr) {
        LOG(INFO) << "Waiting for reloc computation";
        WaitForAllRelocComputations();
        CHECK(reloc_work_queue_ == nullptr);
    }
    WaitForAllComputations();
    absl::MutexLock locker(&work_queue_mutex_);
    CHECK(work_queue_ == nullptr);
    LOG(INFO) << "stop success";
    return true;
}

NodeId BundleAdjustment::AddLocalizationNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>
        &insertion_submaps) {
    // append node which update pose up to last optimized node
    const transform::Rigid3d optimized_pose(
        GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
    LOG_EVERY_N(INFO, 100) << "newest pose is: "
                           << optimized_pose.DebugString();
    NodeId node_id(0, 0);
    if (reloc_work_queue_ == nullptr && is_reloc_) {
        constraint_builder_.SetLocalizationMode(false);
        node_id = AppendNode(constant_data, trajectory_id, insertion_submaps,
                             optimized_pose);
    } else if (not is_reloc_) {
        absl::MutexLock locker(&pose_graph_mutex_);
        node_id = data_.trajectory_nodes.Append(
            trajectory_id, TrajectoryNode{constant_data, optimized_pose});
        constraint_builder_.SetLocalizationMode(true);
        if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
                    ->data.submap != insertion_submaps.back()) {
            const SubmapId submap_id =
                data_.submap_data.Append(trajectory_id, InternalSubmapData());
            data_.submap_data.at(submap_id).submap = insertion_submaps.back();
        }
    }
    // if the front of submaps finished?
    const bool newly_finished_submap =
        insertion_submaps.front()->insertion_finished();
    // Relocalization Phase, Add Reloc Node to find Reloc constraints
    // Normal location phase
    if (not is_reloc_) {
        {
            absl::MutexLock locker(&num_reloc_nodes_mutex_);
            ++num_reloc_nodes_;
            LOG(INFO) << "Nodes in bundle adjustment is: " << num_reloc_nodes_;
        }
        AddRelocWorkItem([=]() LOCKS_EXCLUDED(pose_graph_mutex_) {
            return ComputeConstraintsForRelocNode(node_id, insertion_submaps,
                                                  newly_finished_submap);
        });
    } else {
        if (reloc_work_queue_ == nullptr) {
            LOG(INFO) << "Add Localization Node Here";
            AddWorkItem([=]() LOCKS_EXCLUDED(pose_graph_mutex_) {
                return ComputeConstraintsForNode(node_id, insertion_submaps,
                                                 newly_finished_submap);
            });
        }
    }
    return node_id;
}
// TO_DO(feixiang zeng) : Lack of initial submap global pose
NodeId BundleAdjustment::AddFirstRelocNode(
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>
        &insertion_submaps) {
    LOG(INFO) << "Add first node";
    CHECK(!insertion_submaps.empty());
    CHECK(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id) > 0);
    // trim the first node used for relocalization
    auto node_to_move = data_.trajectory_nodes.BeginOfTrajectory(trajectory_id);
    auto transformed_point_cloud = sensor::TransformPointCloud(
        node_to_move->data.constant_data->filtered_gravity_aligned_point_cloud,
        (node_to_move->data.global_pose *
         node_to_move->data.constant_data->local_pose.inverse())
            .cast<float>());
    std::shared_ptr<const pose_graph::optimization::TrajectoryNode::Data>
        constant_data = std::make_shared<
            const pose_graph::optimization::TrajectoryNode::Data>(
            pose_graph::optimization::TrajectoryNode::Data{
                node_to_move->data.time(),
                node_to_move->data.global_pose.rotation(),
                transformed_point_cloud, node_to_move->data.global_pose});
    std::shared_ptr<const pose_graph::optimization::TrajectoryNode> node =
        std::make_shared<const pose_graph::optimization::TrajectoryNode>(
            pose_graph::optimization::TrajectoryNode{
                constant_data, constant_data->local_pose});
    {
        std::vector<Constraint> constraint_retain;
        for (auto &constraint : data_.constraints) {
            if (constraint.node_id.trajectory_id != trajectory_id) {
                constraint_retain.push_back(constraint);
            }
        }
        data_.constraints = constraint_retain;
        auto id = data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)->id;
        optimization_problem_->TrimTrajectoryNode(id);
        data_.trajectory_nodes.Trim(id);
    }
    AddNode(constant_data, trajectory_id, insertion_submaps);
    return data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)->id;
}

bool BundleAdjustment::FindRelocConstraints(
    std::shared_ptr<const TrajectoryNode> constant_data, int trajectory_id,
    transform::Rigid3d *pose_after_optimized) {
    // pose reloc from vision
    is_working_ = true;
    transform::Rigid3d optimized_pose;
    NodeId node_id(trajectory_id, 0);
    {
        absl::MutexLock locker(&num_reloc_nodes_mutex_);
        ++num_reloc_nodes_;
    }
    node_id = data_.trajectory_nodes.Append(trajectory_id, *constant_data);
    ComputeRelocConstraintsForNode(node_id);
    if (found_reloc_constraints_) {
        auto id = std::prev(optimization_problem_->node_data().EndOfTrajectory(
                                trajectory_id))
                      ->id;
        optimized_pose = data_.trajectory_nodes.at(id).global_pose;
        LOG(INFO) << "optimized pose is: " << optimized_pose.DebugString();
        *pose_after_optimized = optimized_pose;
    }
    is_working_ = false;
    return found_reloc_constraints_;
}

bool BundleAdjustment::RecoverPoseGraphFromLast(
    int trajectory_id, const mapping::MapById<NodeId, TrajectoryNode> &nodes,
    const mapping::MapById<SubmapId, SubmapData> &submaps,
    const std::vector<Constraint> &constraints) {
    absl::MutexLock locker(&pose_graph_mutex_);
    CHECK(optimization_problem_->submap_data().empty());
    // Let the last pose graph data as frozen
    InternalTrajectoryState state;
    state.state = TrajectoryState::FROZEN;
    state.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;
    data_.trajectories_state.insert(std::make_pair(trajectory_id, state));
    // read the node and submap data from last pose graph
    for (auto submap : submaps) {
        const transform::Rigid2d global_submap_pose_2d =
            transform::Project2D(submap.data.pose);
        data_.submap_data.Insert(submap.id, InternalSubmapData());
        data_.submap_data.at(submap.id).submap = submap.data.submap;
        data_.submap_data.at(submap.id).state = SubmapState::kFinished;
        data_.global_submap_poses_2d.Insert(
            submap.id, SubmapSpec2D{global_submap_pose_2d});
        optimization_problem_->InsertSubmap(submap.id, global_submap_pose_2d);
    }

    for (auto node : nodes) {
        data_.trajectory_nodes.Insert(node.id, node.data);
        const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
            node.data.constant_data->gravity_alignment.inverse());
        optimization_problem_->InsertTrajectoryNode(
            node.id, NodeSpec2D{node.data.constant_data->time,
                                transform::Project2D(
                                    node.data.constant_data->local_pose *
                                    gravity_alignment_inverse),
                                transform::Project2D(node.data.global_pose *
                                                     gravity_alignment_inverse),
                                node.data.constant_data->gravity_alignment});
    }
    // Update pose graph Connectively
    for (auto constraint : constraints) {
        CHECK(data_.trajectory_nodes.Contains(constraint.node_id));
        CHECK(data_.submap_data.Contains(constraint.submap_id));
        CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
              nullptr);
        CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
            CHECK(data_.submap_data.at(constraint.submap_id)
                      .node_ids.emplace(constraint.node_id)
                      .second);
        }
        data_.constraints.push_back(constraint);
    }
    InternalTrajectoryState new_trajectory_state;
    new_trajectory_state.state = TrajectoryState::ACTIVE;
    new_trajectory_state.deletion_state =
        InternalTrajectoryState::DeletionState::NORMAL;
    int new_trajectory_id = trajectory_id + 1;
    data_.trajectories_state[new_trajectory_id] = new_trajectory_state;
    return true;
}

void BundleAdjustment::RunOptimization() {
    // have no submap data
    if (optimization_problem_->submap_data().empty()) {
        return;
    }
    LOG(INFO) << "optimization run";
    // no other thread is accessing the optimization problem
    // constraints when executing the solve. solve is time consuming
    // so not take mutex before solve to avoid blocking foreground processing
    optimization_problem_->Solve(data_.constraints, GetTrajectoryStates());

    // update pose info in pose graph from optimization problem after Solve
    absl::MutexLock locker(&pose_graph_mutex_);
    const auto &submap_data = optimization_problem_->submap_data();
    const auto &node_data = optimization_problem_->node_data();
    // in our system only one trajectory actually
    for (const int trajectory_id : node_data.trajectory_ids()) {
        for (const auto &node : node_data.trajectory(trajectory_id)) {
            auto &mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
            mutable_trajectory_node.global_pose =
                transform::Embed3D(node.data.global_pose_2d) *
                transform::Rigid3d::Rotation(
                    mutable_trajectory_node.constant_data->gravity_alignment);
        }

        // Extrapolate all point cloud poses that were not included in
        // 'optimization problem' yet still have node join Added newly, but not
        // optimized yet
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
            auto &mutable_trajectory_node =
                data_.trajectory_nodes.at(node_it->id);
            mutable_trajectory_node.global_pose =
                old_global_to_new_global * mutable_trajectory_node.global_pose;
        }
    }
    data_.global_submap_poses_2d = submap_data;
}

std::vector<SubmapId> BundleAdjustment::InitializeGlobalSubmapPose(
    int trajectory_id, const common::Time &time,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>
        &insertion_submaps) {
    // Initialize Global pose of submaps
    CHECK(!insertion_submaps.empty());
    const auto &submap_data = optimization_problem_->submap_data();
    // in our system, insertion submaps size equals 1 mean the system just start
    // to work
    if (insertion_submaps.size() == 1) {
        if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
            optimization_problem_->AddSubMap(
                trajectory_id,
                transform::Project2D(
                    ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                  trajectory_id) *
                    insertion_submaps[0]->local_pose()));
        }
        CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
        const SubmapId submap_id{trajectory_id, 0};
        CHECK(data_.submap_data.at(submap_id).submap ==
              insertion_submaps.front());
        return {submap_id};
    }
    CHECK_EQ(2, insertion_submaps.size());
    const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
    CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
    const SubmapId last_submap_id = std::prev(end_it)->id;
    // in this case, the last submap in pose graph data euqals
    // insertion_submaps.front() and insertion_submaps.back() is new
    // the last submap in optimization problem equals insertion submaps front,
    // means insertion map back is newly generated, add it into optimization
    if (data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.front()) {
        const auto &first_submap_pose =
            submap_data.at(last_submap_id).global_pose;
        optimization_problem_->AddSubMap(
            trajectory_id,
            first_submap_pose *
                transform::Project2D(
                    insertion_submaps[0]->local_pose().inverse() *
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

void BundleAdjustment::AddWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
    {
        absl::MutexLock locker(&work_queue_mutex_);
        if (work_queue_ == nullptr) {
            work_queue_ = std::make_unique<WorkQueue>();
            auto task = std::make_unique<common::Task>();
            task->SetWorkItem([this]() { DrainWorkQueue(); });
            thread_pool_->Schedule(std::move(task));
        }
        const auto now = std::chrono::steady_clock::now();
        work_queue_->push_back({now, work_item});
    }
}

void BundleAdjustment::AddRelocWorkItem(
    const std::function<WorkItem::Result()> &work_item) {
    {
        absl::MutexLock locker(&reloc_work_queue_mutex_);
        if (reloc_work_queue_ == nullptr) {
            reloc_work_queue_ = std::make_unique<WorkQueue>();
            auto task = std::make_unique<common::Task>();
            task->SetWorkItem([this]() { DrainRelocWorkQueue(); });
            thread_pool_->Schedule(std::move(task));
        }
        const auto now = std::chrono::steady_clock::now();
        reloc_work_queue_->push_back({now, work_item});
    }
}

void BundleAdjustment::DrainRelocWorkQueue() {
    std::function<WorkItem::Result()> work_item;
    {
        absl::MutexLock locker(&reloc_work_queue_mutex_);
        if (reloc_work_queue_->empty()) {
            reloc_work_queue_.reset();
            LOG(INFO) << "reloc work queue empty";
            return;
        }
        work_item = reloc_work_queue_->front().task;
        reloc_work_queue_->pop_front();
        LOG(INFO) << "Pop Reloc Work Queue";
    }
    work_item();
    constraint_builder_.RelocWhenDone(
        [this](const ConstraintBuilder::Result &result) {
            HandleRelocWorkQueue(result);
        });
}

void BundleAdjustment::DrainWorkQueue() {
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
            work_queue_size = work_queue_->size();
        }
        process_work_queue =
            work_item() == WorkItem::Result::kDoNotRunOptimization;
        if (process_work_queue) {
            LOG_EVERY_N(INFO, 100)
                << "Still not trigger optimization, process task in work queue."
                << " parameter nodes is: " << param_.optimize_every_n_nodes;
        }
    }
    LOG_EVERY_N(INFO, 50) << "Remaining work items in queue: "
                          << work_queue_size;
    constraint_builder_.WhenDone(
        [this](const ConstraintBuilder::Result &result) {
            HandleWorkQueue(result);
        });
}

bool BundleAdjustment::LoopVerify(const optimization::Constraint &constraint) {
    // Verify loop

}

void BundleAdjustment::HandleRelocWorkQueue(
    const ConstraintBuilder::Result &result) {
    LOG(INFO) << "Reloc result in: " << result.size();
    // if (is_reloc_)
    //     DrainRelocWorkQueue();
    // Verify The Reloc Pose if vision reloc pose is
    if (result.size() > 0) {
        LOG(INFO) << "Found Reloc Constraints, Which Size is: "
                  << result.size();
        if (param_.need_vision_verify) {
            std::vector<NodeId> ids;
            std::map<NodeId, transform::Rigid3d> id_global_pose;
            bool pose_arrive = true;
            std::chrono::time_point<std::chrono::high_resolution_clock> start =
                std::chrono::high_resolution_clock::now();
            while (ids.empty()) {
                LOG(INFO)
                    << "Still No Correspond Node Id Vision Constraint Arrive";
                for (auto &constraint : result) {
                    auto node_id = constraint.node_id;
                    if (vision_id_pose_.Contains(node_id)) {
                        ids.push_back(node_id);
                        auto global_pose =
                            constraint.pose.zbar_ij *
                            data_.submap_data.at(constraint.submap_id)
                                .submap->local_pose();
                        id_global_pose[node_id] = global_pose;
                    }
                }
                auto duration =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now() - start)
                        .count();
                if (duration * 0.000001 > 1) {
                    pose_arrive = false;
                    break;
                }
            }
            if (pose_arrive) {
                for (auto id : ids) {
                    auto global_pose_laser = id_global_pose[id];
                    auto global_pose_vision = vision_id_pose_.at(id);
                    // compare two pose to verify
                }
            } else {
                LOG(INFO) << "Found Reloc Constraints";
                // use the laser reloc constraint directly

                {
                    absl::MutexLock locker(&pose_graph_mutex_);
                    data_.constraints.insert(data_.constraints.end(),
                                             result.begin(), result.end());
                }
                RunOptimization();

                is_reloc_ = true;
                constraint_builder_.SetIsReloc();
            }
        } else {
            // insert constraint directly, run optimiation
            {
                absl::MutexLock locker(&pose_graph_mutex_);
                data_.constraints.insert(data_.constraints.end(),
                                         result.begin(), result.end());
            }
            RunOptimization();
            is_reloc_ = true;
            constraint_builder_.SetIsReloc();
        }
    }
    DrainRelocWorkQueue();
}

void BundleAdjustment::HandleWorkQueue(
    const ConstraintBuilder::Result &result) {
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        data_.constraints.insert(data_.constraints.end(), result.begin(),
                                 result.end());
    }
    RunOptimization();
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        int trajectory_id;
        for (auto id_state : data_.trajectories_state) {
            if (id_state.second.state == TrajectoryState::ACTIVE) {
                trajectory_id = id_state.first;
            }
        }

        if (param_.max_submaps_maintain > 0) {
            auto submap_num =
                data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id);
            int i = param_.max_submaps_maintain;

            while (submap_num > param_.max_submaps_maintain) {
                CHECK(param_.max_submaps_maintain >= 3)
                    << "Please set 'max submaps maintain' greater equal than 3 "
                       "to make "
                       "sure the submap trimmed is 'insertion finished'";
                LOG(INFO) << "trim submap when submap num is: " << submap_num;
                std::string state =
                    data_.submap_data.BeginOfTrajectory(trajectory_id)
                                ->data.state == SubmapState::kFinished
                        ? "finished"
                        : "unfinished";
                LOG(INFO) << "state of submap is: " << state;
                LOG(INFO)
                    << "submap id is: "
                    << data_.submap_data.EndOfTrajectory(trajectory_id - 1)->id;
                TrimSubmap(
                    data_.submap_data.EndOfTrajectory(trajectory_id - 1)->id);
                --submap_num;
            }
        }

        std::lock_guard<std::mutex> lk_node(
            num_nodes_since_loop_closure_mutex_);
        num_nodes_since_last_loop_closure_ = 0;
    }
    DrainWorkQueue();
}

// compute pose graph newly optimized global pose, local to global
transform::Rigid3d
BundleAdjustment::GetLocalToGlobalTransform(const int trajectory_id) const {
    absl::MutexLock locker(&pose_graph_mutex_);
    return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                         trajectory_id);
}

// the global submap pose is changed in optimization problem first and then
// changed in pose graph
transform::Rigid3d BundleAdjustment::ComputeLocalToGlobalTransform(
    const mapping::MapById<SubmapId, SubmapSpec2D> &global_submap_poses,
    int trajectory_id) const {
    auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
    auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
    // there is no pose data in data_
    if (begin_it == end_it) {
        // we do not have initial poses from other trajectory, we also do not do
        // the connective trajectory work
        return transform::Rigid3d::Identity();
    }
    const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
    LOG_EVERY_N(INFO, 100) << "global submap pose is: "
                           << global_submap_poses.at(last_optimized_submap_id)
                                  .global_pose.DebugString();
    return transform::Embed3D(
               global_submap_poses.at(last_optimized_submap_id).global_pose) *
           data_.submap_data.at(last_optimized_submap_id)
               .submap->local_pose()
               .inverse();
}

NodeId BundleAdjustment::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    int trajectory_id,
    const std::vector<std::shared_ptr<const mapping::Submap2D>>
        &insertion_submaps,
    const transform::Rigid3d &optimized_pose) {
    absl::MutexLock locker(&pose_graph_mutex_);
    const NodeId node_id = data_.trajectory_nodes.Append(
        trajectory_id, TrajectoryNode{constant_data, optimized_pose});
    ++data_.num_trajectory_nodes;
    // if data_ have no submap data or the back of insertion submap not included
    // in data_, just append it
    if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
        std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
                ->data.submap != insertion_submaps.back()) {
        const SubmapId submap_id =
            data_.submap_data.Append(trajectory_id, InternalSubmapData());
        data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    }

    return node_id;
}

void BundleAdjustment::RelocConstraintsWorkHandle(
    const ConstraintBuilder::Result &result) {
    if (result.size() > 0) {
        LOG(INFO) << "Found Reloc Constraints, Which Size is: "
                  << result.size();
        found_reloc_constraints_ = true;
        data_.constraints.insert(data_.constraints.end(), result.begin(),
                                 result.end());
    }
}

void BundleAdjustment::ComputeRelocConstraint(const NodeId &node_id,
                                              const SubmapId &submap_id) {
    const TrajectoryNode::Data *constant_data;
    const mapping::Submap2D *submap;
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
        if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
            return;
        }
        constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
        submap = static_cast<const mapping::Submap2D *>(
            data_.submap_data.at(submap_id).submap.get());
    }
    const transform::Rigid2d initial_relative_pose =
        transform::Rigid2d::Identity();
    constraint_builder_.FindRelocLoopConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
}

// Compute reloc constraints for node
bool BundleAdjustment::ComputeRelocConstraintsForNode(const NodeId &node_id) {
    // Submaps from last pose graph

    for (const auto &submap_id_data : data_.submap_data) {
        InternalTrajectoryState state =
            data_.trajectories_state[submap_id_data.id.trajectory_id];
        if (state.state == TrajectoryState::FROZEN) {
            const transform::Rigid2d initial_relative_pose =
                optimization_problem_->submap_data()
                    .at(submap_id_data.id)
                    .global_pose.inverse() *
                transform::Project2D(
                    data_.trajectory_nodes.at(node_id).global_pose);
            const mapping::Submap2D *submap =
                static_cast<const mapping::Submap2D *>(
                    submap_id_data.data.submap.get());
            constraint_builder_.FindRelocLoopConstraint(
                submap_id_data.id, submap, node_id,
                data_.trajectory_nodes.at(node_id).constant_data.get(),
                initial_relative_pose);
        }
    }
    constraint_builder_.NotifyEndOfOneRelocNode();

    // Wait until all task finish
    while (constraint_builder_.GetNumFinishedRelocNodes() != num_reloc_nodes_) {
        usleep(1000);
    }

    // callback all reloc constraint result and judge is reloc successfully
    constraint_builder_.RelocWhenDone(
        [this](const ConstraintBuilder::Result result) {
            RelocConstraintsWorkHandle(result);
        });

    if (found_reloc_constraints_) {
        // if found the constraint between old and new node, add to optimizaiton
        // problem and optimize it
        auto constant_data = data_.trajectory_nodes.at(node_id).constant_data;
        const transform::Rigid2d local_pose_2d = transform::Project2D(
            constant_data->local_pose *
            transform::Rigid3d::Rotation(
                constant_data->gravity_alignment.inverse()));

        // Initial node for relocalization, view global pose as local
        optimization_problem_->AddTrajectoryNode(
            node_id.trajectory_id,
            NodeSpec2D{constant_data->time, local_pose_2d, local_pose_2d,
                       constant_data->gravity_alignment});
        RunOptimization();

    } else {
        // if not reloc successfully, useless node should be trimmed
        data_.trajectory_nodes.Trim(node_id);
    }

    return found_reloc_constraints_;
}
WorkItem::Result BundleAdjustment::ComputeConstraintsForRelocNode(
    const NodeId &node_id,
    std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
    bool newly_finished_submap) {

    std::vector<SubmapId> submap_ids;
    std::set<NodeId> newly_finished_submap_node_ids;
    // Add Data to pose_graph data, same as ComputeConstraintsForNode
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        const auto &constant_data =
            data_.trajectory_nodes.at(node_id).constant_data;
        submap_ids = InitializeGlobalSubmapPose(
            node_id.trajectory_id, constant_data->time, insertion_submaps);
        CHECK_EQ(insertion_submaps.size(), submap_ids.size());
        const SubmapId matching_id = submap_ids.front();
        // gravity aligned pose which without angle info
        const transform::Rigid2d local_pose_2d = transform::Project2D(
            constant_data->local_pose *
            transform::Rigid3d::Rotation(
                constant_data->gravity_alignment.inverse()));
        LOG_EVERY_N(INFO, 50)
            << "global pose is: "
            << optimization_problem_->submap_data()
                   .at(matching_id)
                   .global_pose.DebugString()
            << " id of submap is: " << matching_id.submap_index
            << "gravity alignment is: "
            << constant_data->gravity_alignment.toRotationMatrix();
        // global pose 2d is calculate pose of node from submap local pose
        // optimized
        const transform::Rigid2d global_pose_2d =
            optimization_problem_->submap_data().at(matching_id).global_pose *
            transform::Project2D(insertion_submaps.front()->local_pose())
                .inverse() *
            local_pose_2d;
        LOG_EVERY_N(INFO, 50)
            << "global pose is: " << global_pose_2d.DebugString()
            << " local pose is: " << local_pose_2d.DebugString();
        optimization_problem_->AddTrajectoryNode(
            matching_id.trajectory_id,
            NodeSpec2D{constant_data->time, local_pose_2d, global_pose_2d,
                       constant_data->gravity_alignment});
        for (size_t i = 0; i < insertion_submaps.size(); ++i) {
            const SubmapId submap_id = submap_ids[i];
            CHECK(data_.submap_data.at(submap_id).state ==
                  SubmapState::kNoConstraintSearch);
            data_.submap_data.at(submap_id).node_ids.emplace(node_id);
            const transform::Rigid2d constraint_transform =
                transform::Project2D(insertion_submaps[i]->local_pose())
                    .inverse() *
                local_pose_2d;
            data_.constraints.push_back(
                Constraint{submap_id,
                           node_id,
                           {transform::Embed3D(constraint_transform),
                            param_.matcher_translation_weight,
                            param_.matcher_rotation_weight},
                           Constraint::INTRA_SUBMAP});
        }
        if (newly_finished_submap) {
            const SubmapId newly_finished_submap_id = submap_ids.front();
            InternalSubmapData &finished_submap_data =
                data_.submap_data.at(newly_finished_submap_id);
            CHECK(finished_submap_data.state ==
                  SubmapState::kNoConstraintSearch);
            finished_submap_data.state = SubmapState::kFinished;
            newly_finished_submap_node_ids = finished_submap_data.node_ids;
        }
    }
    // INTER Constraint search on submap from last trajectory id
    for (const auto &submap_id_data : data_.submap_data) {
        InternalTrajectoryState state =
            data_.trajectories_state[submap_id_data.id.trajectory_id];
        if (state.state == TrajectoryState::FROZEN) {
            ComputeRelocConstraint(node_id, submap_id_data.id);
        }
    }
    LOG(INFO) << "Compute reloc constraint";
    constraint_builder_.NotifyEndOfOneRelocNode();
    if (not found_reloc_constraints_)
        return WorkItem::Result::kDoNotRunOptimization;
    return WorkItem::Result::kRunOptimization;
}

// TO_DO(Feixiang Zeng) : Add match full submap strategy when reloc
WorkItem::Result BundleAdjustment::ComputeConstraintsForNode(
    const NodeId &node_id,
    std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps,
    bool newly_finished_submap) {
    std::vector<SubmapId> submap_ids;
    std::vector<SubmapId> finished_submap_ids;
    std::set<NodeId> newly_finished_submap_node_ids;
    {
        absl::MutexLock locker(&pose_graph_mutex_);

        const auto &constant_data =
            data_.trajectory_nodes.at(node_id).constant_data;
        submap_ids = InitializeGlobalSubmapPose(
            node_id.trajectory_id, constant_data->time, insertion_submaps);
        CHECK_EQ(insertion_submaps.size(), submap_ids.size());
        const SubmapId matching_id = submap_ids.front();
        // gravity aligned pose which without angle info
        const transform::Rigid2d local_pose_2d = transform::Project2D(
            constant_data->local_pose *
            transform::Rigid3d::Rotation(
                constant_data->gravity_alignment.inverse()));
        LOG_EVERY_N(INFO, 50)
            << "global pose is: "
            << optimization_problem_->submap_data()
                   .at(matching_id)
                   .global_pose.DebugString()
            << " id of submap is: " << matching_id.submap_index
            << "gravity alignment is: "
            << constant_data->gravity_alignment.toRotationMatrix();
        // global pose 2d is calculate pose of node from submap local pose
        // optimized
        const transform::Rigid2d global_pose_2d =
            optimization_problem_->submap_data().at(matching_id).global_pose *
            transform::Project2D(insertion_submaps.front()->local_pose())
                .inverse() *
            local_pose_2d;
        LOG_EVERY_N(INFO, 50)
            << "global pose is: " << global_pose_2d.DebugString()
            << " local pose is: " << local_pose_2d.DebugString();
        optimization_problem_->AddTrajectoryNode(
            matching_id.trajectory_id,
            NodeSpec2D{constant_data->time, local_pose_2d, global_pose_2d,
                       constant_data->gravity_alignment});
        for (size_t i = 0; i < insertion_submaps.size(); ++i) {
            const SubmapId submap_id = submap_ids[i];
            CHECK(data_.submap_data.at(submap_id).state ==
                  SubmapState::kNoConstraintSearch);
            data_.submap_data.at(submap_id).node_ids.emplace(node_id);
            const transform::Rigid2d constraint_transform =
                transform::Project2D(insertion_submaps[i]->local_pose())
                    .inverse() *
                local_pose_2d;
            data_.constraints.push_back(
                Constraint{submap_id,
                           node_id,
                           {transform::Embed3D(constraint_transform),
                            param_.matcher_translation_weight,
                            param_.matcher_rotation_weight},
                           Constraint::INTRA_SUBMAP});
        }
        // INTER Constraint only search on finished submap
        for (const auto &submap_id_data : data_.submap_data) {
            if (submap_id_data.data.state == SubmapState::kFinished) {
                CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
                finished_submap_ids.emplace_back(submap_id_data.id);
            }
        }

        if (newly_finished_submap) {
            const SubmapId newly_finished_submap_id = submap_ids.front();
            InternalSubmapData &finished_submap_data =
                data_.submap_data.at(newly_finished_submap_id);
            CHECK(finished_submap_data.state ==
                  SubmapState::kNoConstraintSearch);
            finished_submap_data.state = SubmapState::kFinished;
            newly_finished_submap_node_ids = finished_submap_data.node_ids;
        }
    }
    // compute constraint for each node and finished submap
    for (const auto &submap_id : finished_submap_ids) {
        ComputeConstraint(node_id, submap_id);
    }
    if (newly_finished_submap) {
        const SubmapId newly_finished_submap_id = submap_ids.front();

        // new finished map, should add constraints for old nodes
        for (const auto &node_id_data : optimization_problem_->node_data()) {
            const NodeId &node_id_b = node_id_data.id;
            if (newly_finished_submap_node_ids.count(node_id_b) == 0 &&
                node_id.trajectory_id == node_id_b.trajectory_id) {
                ComputeConstraint(node_id_b, newly_finished_submap_id);
            }
        }
    }
    constraint_builder_.NotifyEndOfNode();
    {
        std::lock_guard<std::mutex> lk(num_nodes_since_loop_closure_mutex_);
        ++num_nodes_since_last_loop_closure_;
    }
    if (param_.optimize_every_n_nodes > 0 &&
        num_nodes_since_last_loop_closure_ > param_.optimize_every_n_nodes) {
        return WorkItem::Result::kRunOptimization;
    }
    return WorkItem::Result::kDoNotRunOptimization;
}

void BundleAdjustment::ComputeConstraint(const NodeId &node_id,
                                         const SubmapId &submap_id) {
    const TrajectoryNode::Data *constant_data;
    const mapping::Submap2D *submap;
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
        if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
            return;
        }
        constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
        submap = static_cast<const mapping::Submap2D *>(
            data_.submap_data.at(submap_id).submap.get());
    }
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    LOG_EVERY_N(INFO, 100) << "submap global pose is: "
                           << optimization_problem_->submap_data()
                                  .at(submap_id)
                                  .global_pose.DebugString()
                           << " node global pose is: "
                           << optimization_problem_->node_data()
                                  .at(node_id)
                                  .global_pose_2d.DebugString();
    constraint_builder_.AddLocalLoopConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
}

std::map<int, TrajectoryState> BundleAdjustment::GetTrajectoryStates() const {
    std::map<int, TrajectoryState> trajectories_state;
    absl::MutexLock locker(&pose_graph_mutex_);
    for (const auto &it : data_.trajectories_state) {
        trajectories_state[it.first] = it.second.state;
    }
    return trajectories_state;
}

void BundleAdjustment::RunFinalOptimization() {
    {
        AddWorkItem([this]() LOCKS_EXCLUDED(pose_graph_mutex_) {
            absl::MutexLock locker(&pose_graph_mutex_);
            optimization_problem_->SetMaxNumIterations(
                param_.max_num_final_iterations);
            return WorkItem::Result::kRunOptimization;
        });
        AddWorkItem([this]() {
            absl::MutexLock locker(&pose_graph_mutex_);
            optimization_problem_->SetMaxNumIterations(
                param_.optimization_param.max_num_iterations);
            return WorkItem::Result::kDoNotRunOptimization;
        });
    }
    WaitForAllComputations();
}

void BundleAdjustment::WaitForAllRelocComputations() {
    const int num_finished_reloc_nodes_at_start =
        constraint_builder_.GetNumFinishedRelocNodes();
    int num_reloc_trajectory_nodes;
    {
        absl::MutexLock locker(&num_reloc_nodes_mutex_);
        num_reloc_trajectory_nodes = num_reloc_nodes_;
    }

    auto report_progress = [this, num_reloc_trajectory_nodes,
                            num_finished_reloc_nodes_at_start]() {
        // Log progress on nodes only when we are actually processing nodes.
        if (num_reloc_trajectory_nodes != num_finished_reloc_nodes_at_start) {
            std::ostringstream progress_info;
            progress_info
                << "Optimizing: " << std::fixed << std::setprecision(1)
                << 100. *
                       (constraint_builder_.GetNumFinishedRelocNodes() -
                        num_finished_reloc_nodes_at_start) /
                       (num_reloc_trajectory_nodes -
                        num_finished_reloc_nodes_at_start)
                << "%...";
            std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
        }
    };

    {
        const auto predicate =
            [this]() EXCLUSIVE_LOCKS_REQUIRED(reloc_work_queue_mutex_) {
                return reloc_work_queue_ == nullptr;
            };
        absl::MutexLock locker(&reloc_work_queue_mutex_);
        while (!reloc_work_queue_mutex_.AwaitWithTimeout(
            absl::Condition(&predicate),
            absl::FromChrono(common::FromSeconds(1.)))) {
            LOG(INFO) << "IN WHILE";
            report_progress();
        }
    }

    CHECK_EQ(constraint_builder_.GetNumFinishedRelocNodes(),
             num_reloc_trajectory_nodes);
    std::cout << "\r\x1b[RelocThread: Done.     " << std::endl;
}

void BundleAdjustment::WaitForAllComputations() {
    int num_trajectory_nodes;
    {
        absl::MutexLock locker(&pose_graph_mutex_);
        num_trajectory_nodes = data_.num_trajectory_nodes;
    }

    const int num_finished_nodes_at_start =
        constraint_builder_.GetNumFinishedNodes();

    auto report_progress =
        [this, num_trajectory_nodes, num_finished_nodes_at_start]() {
            // Log progress on nodes only when we are actually processing nodes.
            if (num_trajectory_nodes != num_finished_nodes_at_start) {
                std::ostringstream progress_info;
                progress_info
                    << "Optimizing: " << std::fixed << std::setprecision(1)
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
            LOG(INFO) << "IN While";
            report_progress();
        }
    }

    // Now wait for any pending constraint computations to finish.
    absl::MutexLock locker(&pose_graph_mutex_);
    bool notification = false;
    constraint_builder_.WhenDone(
        [this, &notification](const ConstraintBuilder::Result &result)
            LOCKS_EXCLUDED(pose_graph_mutex_) {
                absl::MutexLock locker(&pose_graph_mutex_);
                data_.constraints.insert(data_.constraints.end(),
                                         result.begin(), result.end());
                notification = true;
            });
    const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
        return notification;
    };
    while (!pose_graph_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(1.)))) {
        LOG(INFO) << "IN While";
        report_progress();
    }
    CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
    std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

TrajectoryNode BundleAdjustment::GetLatestNodeData(int trajectory_id) const {
    absl::MutexLock locker(&pose_graph_mutex_);
    auto node_it = data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
    return data_.trajectory_nodes.at(node_it->id);
}

PoseGraphData BundleAdjustment::pose_graph_data() const {
    absl::MutexLock locker(&pose_graph_mutex_);
    return data_;
}

SubmapData
BundleAdjustment::GetSubmapDataUnderLock(const SubmapId &submap_id) const {
    const auto it = data_.submap_data.find(submap_id);
    if (it == data_.submap_data.end()) {
        return {};
    }
    auto submap = it->data.submap;
    if (data_.global_submap_poses_2d.Contains(submap_id)) {
        // we already have optimized pose
        return {submap,
                transform::Embed3D(
                    data_.global_submap_poses_2d.at(submap_id).global_pose)};
    }
    // still not optimized, extrapolate it
    return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                  submap_id.trajectory_id) *
                        submap->local_pose()};
}

mapping::MapById<SubmapId, SubmapData>
BundleAdjustment::GetSubmapDataUnderLock() const {
    absl::MutexLock locker(&pose_graph_mutex_);
    mapping::MapById<SubmapId, SubmapData> submaps;
    for (const auto &submap_id_data : data_.submap_data) {
        submaps.Insert(submap_id_data.id,
                       GetSubmapDataUnderLock(submap_id_data.id));
    }
    return submaps;
}

mapping::MapById<NodeId, TrajectoryNode>
BundleAdjustment::GetTrajectoryNodes() const {
    absl::MutexLock locker(&pose_graph_mutex_);
    return data_.trajectory_nodes;
}

// this function should be used under pose graph locker outside
void BundleAdjustment::TrimSubmap(const SubmapId &id) {
    CHECK(data_.submap_data.at(id).state == SubmapState::kFinished);
    std::set<NodeId> nodes_to_retain;
    for (const auto &submap_data : data_.submap_data) {
        if (submap_data.id != id) {
            nodes_to_retain.insert(submap_data.data.node_ids.begin(),
                                   submap_data.data.node_ids.end());
        }
    }
    // remove all node which internal submap
    std::set<NodeId> nodes_to_remove;
    std::set_difference(
        data_.submap_data.at(id).node_ids.begin(),
        data_.submap_data.at(id).node_ids.end(), nodes_to_retain.begin(),
        nodes_to_retain.end(),
        std::inserter(nodes_to_remove, nodes_to_remove.begin()));

    // remove all constraints related to submap
    std::vector<Constraint> constraints;
    for (const Constraint &constraint : data_.constraints) {
        if (constraint.submap_id != id) {
            constraints.push_back(constraint);
        }
    }
    data_.constraints = std::move(constraints);

    // Remove all "data_.constraints" related to "nodes_to_remove"
    // Nodes will be removed may have 'INTER' constraint with other submap
    {
        std::vector<Constraint> constraints_after_remove_inter;
        std::set<SubmapId> other_submap_ids_losing_constraints;
        for (const Constraint &constraint : data_.constraints) {
            if (nodes_to_remove.count(constraint.node_id) == 0) {
                constraints_after_remove_inter.push_back(constraint);
            } else {
                other_submap_ids_losing_constraints.insert(
                    constraint.submap_id);
            }
        }

        data_.constraints = std::move(constraints_after_remove_inter);

        for (const Constraint &constraint : data_.constraints) {
            if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
                continue;
            } else if (other_submap_ids_losing_constraints.count(
                           constraint.submap_id)) {
                other_submap_ids_losing_constraints.erase(constraint.submap_id);
            }
        }

        for (const SubmapId &submap_id : other_submap_ids_losing_constraints) {
            constraint_builder_.DeleteScanMatcher(submap_id);
        }
    }

    CHECK(data_.submap_data.at(id).state == SubmapState::kFinished);
    data_.submap_data.Trim(id);
    constraint_builder_.DeleteScanMatcher(id);
    optimization_problem_->TrimSubmap(id);

    for (const NodeId &node_id : nodes_to_remove) {
        data_.trajectory_nodes.Trim(node_id);
        optimization_problem_->TrimTrajectoryNode(node_id);
    }
}

// Submap Data {global pose, submap 2d}
protos::mapping::proto::PoseGraph
BundleAdjustment::ToProto(int trajectory_id,
                          bool include_unfinished_submaps) const {
    protos::mapping::proto::PoseGraph proto;
    proto.set_trajectory_id(trajectory_id);
    // Save the data of submap
    std::set<SubmapId> unfinished_submaps;
    for (const auto &submap_id_data : GetSubmapDataUnderLock()) {
        if (!include_unfinished_submaps &&
            !submap_id_data.data.submap->insertion_finished()) {
            unfinished_submaps.insert(submap_id_data.id);
            continue;
        }
        protos::mapping::proto::SubmapData submap_data;
        protos::transform::proto::Rigid3d global_pose;
        global_pose = transform::ToProto(transform::Embed3D(
            data_.global_submap_poses_2d.at(submap_id_data.id).global_pose));
        *submap_data.mutable_submap() =
            submap_id_data.data.submap->ToProto(true);
        *submap_data.mutable_global_pose() = global_pose;
        submap_data.mutable_id()->set_submap_index(
            submap_id_data.id.submap_index);
        submap_data.mutable_id()->set_trajectory_id(
            submap_id_data.id.trajectory_id);
        *proto.add_submaps() = submap_data;
        LOG(INFO) << "proto saved";
    }

    auto constraints_copy = data_.constraints;
    std::set<NodeId> orphaned_nodes;
    proto.mutable_constraint()->Reserve(constraints_copy.size());
    for (auto it = constraints_copy.begin(); it != constraints_copy.end();) {
        if (!include_unfinished_submaps &&
            unfinished_submaps.count(it->submap_id) > 0) {
            // skip these constraints that refer to unfinished submap
            orphaned_nodes.insert(it->node_id);
            it = constraints_copy.erase(it);
            continue;
        }
        *proto.add_constraint() =
            cartographer::pose_graph::optimization::ToProto(*it);
        ++it;
    }

    if (!include_unfinished_submaps) {
        for (const auto &constraint : constraints_copy) {
            orphaned_nodes.erase(constraint.node_id);
        }
    }

    // Save the trajectory nodes
    for (const auto &node_id_data : GetTrajectoryNodes()) {
        auto *const node = proto.add_trajectory();
        node->set_node_index(node_id_data.id.node_index);
        protos::mapping::proto::TrajectoryNodeData data;
        data.set_timestamp(
            common::ToUniversal(node_id_data.data.constant_data->time));
        *data.mutable_gravity_alignment() = transform::ToProto(
            node_id_data.data.constant_data->gravity_alignment);
        *data.mutable_local_pose() =
            transform::ToProto(node_id_data.data.constant_data->local_pose);
        *data.mutable_filtered_gravity_aligned_point_cloud() =
            node_id_data.data.constant_data
                ->filtered_gravity_aligned_point_cloud.ToProto();
        *node->mutable_data() = data;
        *node->mutable_global_pose() =
            transform::ToProto(node_id_data.data.global_pose);
    }
    return proto;
}

} // namespace optimization
} // namespace pose_graph
} // namespace cartographer
