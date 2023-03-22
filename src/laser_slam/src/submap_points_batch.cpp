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
#include <memory>

#include "laser_slam/base_data/submap_points_batch.hpp"

namespace cartographer
{
namespace laser_slam
{
SubmapPointsBatch::SubmapPointsBatch(
  double resolution, int init_width,
  int init_height)
: resolution_(resolution), quit_thread_(false), submap_count_(0),
  origin_(DBL_MAX, DBL_MAX), right_up_corner_(-DBL_MAX, -DBL_MAX),
  thread_(nullptr)
{
  final_map_.info.height = init_height;
  final_map_.info.width = init_width;
  final_map_.info.resolution = resolution;
  final_map_.info.origin.position.x = -init_height * resolution / 2.0;
  final_map_.info.origin.position.y = -init_width * resolution / 2.0;
  final_map_.info.origin.position.z = 0.0;
  final_map_.info.origin.orientation.w = 1.0;
  final_map_.info.origin.orientation.x = 0.0;
  final_map_.info.origin.orientation.y = 0.0;
  final_map_.info.origin.orientation.z = 0.0;
  final_map_.data.resize(init_height * init_width);
}
SubmapPointsBatch::~SubmapPointsBatch() {}

bool SubmapPointsBatch::StartThread()
{
  if (thread_) {
    LOG(ERROR) << "Realtime Mapping Thread Already exist";
    return false;
  }
  thread_.reset(new std::thread(&SubmapPointsBatch::MapTaskLoop, this));
  return true;
}

bool SubmapPointsBatch::QuitThread()
{
  if (thread_ != nullptr) {
    quit_thread_ = true;
    tasks_condvar_.notify_all();
    if (thread_->joinable()) {
      thread_->join();
    }
    thread_.reset();
  }
  return true;
}

void SubmapPointsBatch::AddSubmap(
  const mapping::SubmapId & id,
  const std::shared_ptr<const mapping::Submap> & submap)
{
  std::unique_lock<std::mutex> lk(job_mx_);
  task_queue_.push(Task(id, submap));
  tasks_condvar_.notify_all();
}

void SubmapPointsBatch::AddRangeData(
  const mapping::NodeId & id,
  const sensor::RangeData & range_data)
{
  std::unique_lock<std::mutex> lk(job_mx_);
  task_queue_.push(Task(id, range_data));
  tasks_condvar_.notify_all();
}

void SubmapPointsBatch::Flush(
  const mapping::MapById<mapping::SubmapId,
  pose_graph::optimization::SubmapSpec2D>
  & pose_graph_submap)
{
  std::unique_lock<std::mutex> lk(job_mx_);
  task_queue_.push(Task(pose_graph_submap));
  tasks_condvar_.notify_all();
}

void SubmapPointsBatch::Update(const sensor::RangeData & range_data)
{
  if (range_data.returns.empty()) {
    return;
  }
  int width, height, offset_x, offset_y;
  offset_x = 0;
  offset_y = 0;
  auto origin_bk = origin_;
  auto data_bk = final_map_.data;
  auto right_up = right_up_corner_;
  int old_width = final_map_.info.width;
  int old_height = final_map_.info.height;
  for (size_t i = 0; i < range_data.returns.size(); ++i) {
    double x, y;
    x = range_data.returns[i].position.x();
    y = range_data.returns[i].position.y();
    if (x < origin_.x()) {
      origin_.x() = x;
    }
    if (y < origin_.y()) {
      origin_.y() = y;
    }
    if (x > right_up_corner_.x()) {
      right_up_corner_.x() = x;
    }
    if (y > right_up_corner_.y()) {
      right_up_corner_.y() = y;
    }
  }
  width = std::ceil((right_up_corner_.x() - origin_.x()) / resolution_);
  height = std::ceil((right_up_corner_.y() - origin_.y()) / resolution_);
  Eigen::Vector2d offset = origin_bk - origin_;

  offset_x = std::ceil(offset.y() / resolution_);

  offset_y = std::ceil(offset.x() / resolution_);
  if (offset_x < 0) {
    offset_x = 0;
  }
  if (offset_y < 0) {
    offset_y = 0;
  }

  final_map_.data.clear();
  final_map_.data.resize(width * height);
  for (int k = 0; k < width * height; ++k) {
    final_map_.data[k] = 0;
  }
  final_map_.info.height = height;
  final_map_.info.width = width;
  final_map_.info.origin.position.x = origin_.x();
  final_map_.info.origin.position.y = origin_.y();
  if (!first_update_) {
    for (int id_x = 0; id_x < old_height; ++id_x) {
      for (int id_y = 0; id_y < old_width; ++id_y) {
        int new_id_x = id_x + offset_x;
        int new_id_y = id_y + offset_y;
        final_map_.data[new_id_x * width + new_id_y] =
          data_bk[id_x * old_width + id_y];
      }
    }
  }
  for (auto pts : range_data.returns) {
    int index_x = std::ceil((pts.position.y() - origin_.y()) / resolution_);
    int index_y = std::ceil((pts.position.x() - origin_.x()) / resolution_);
    // happen sometimes
    if (index_x < 0) {
      index_x = 0;
    }
    if (index_y < 0) {
      index_y = 0;
    }
    final_map_.data[index_x * width + index_y] = 100;
  }
  first_update_ = false;
  is_grid_ = true;
}

void SubmapPointsBatch::UpdateMap(
  const nav_msgs::msg::OccupancyGrid & new_submap)
{
  Eigen::Vector2d origin(new_submap.info.origin.position.x,
    new_submap.info.origin.position.y);
  origin_.x() = origin_.x() > origin.x() ? origin.x() : origin_.x();
  origin_.y() = origin_.y() > origin.y() ? origin.y() : origin_.y();

  int width = new_submap.info.width;
  int height = new_submap.info.height;
  Eigen::Vector2d right_corner(
    origin.x() + width * new_submap.info.resolution,
    origin.y() + height * new_submap.info.resolution);
  right_up_corner_.x() = right_corner.x() > right_up_corner_.x() ?
    right_corner.x() :
    right_up_corner_.x();
  right_up_corner_.y() = right_corner.y() > right_up_corner_.y() ?
    right_corner.y() :
    right_up_corner_.y();
  int index_x = std::ceil(
    (origin.x() - final_map_.info.origin.position.x) /
    final_map_.info.resolution) -
    1;
  int index_y = std::ceil(
    (origin.y() - final_map_.info.origin.position.y) /
    final_map_.info.resolution) -
    1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (final_map_.data[(y + index_y) * final_map_.info.width + x +
        index_x] == -1)
      {
        final_map_
        .data[(y + index_y) * final_map_.info.width + x + index_x] =
          new_submap.data[y * width + x];
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid SubmapPointsBatch::CropGrid()
{
  int width = (right_up_corner_.x() - origin_.x()) / resolution_;
  int height = (right_up_corner_.y() - origin_.y()) / resolution_;
  nav_msgs::msg::OccupancyGrid new_ros_grid;
  new_ros_grid.info.height = height;
  new_ros_grid.info.width = width;
  new_ros_grid.info.resolution = resolution_;
  new_ros_grid.info.origin.position.x = origin_.x();
  new_ros_grid.info.origin.position.y = origin_.y();
  new_ros_grid.info.origin.position.z = 0.0;
  new_ros_grid.info.origin.orientation.w = 1.0;
  new_ros_grid.info.origin.orientation.x = 0.0;
  new_ros_grid.info.origin.orientation.y = 0.0;
  new_ros_grid.info.origin.orientation.z = 0.0;
  new_ros_grid.data.resize(height * width);
  int index_x = std::ceil(
    (origin_.x() - final_map_.info.origin.position.x) /
    resolution_) -
    1;
  int index_y = std::ceil(
    (origin_.y() - final_map_.info.origin.position.y) /
    resolution_) -
    1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      new_ros_grid.data[y * width + x] =
        final_map_
        .data[(y + index_y) * final_map_.info.width + x + index_x];
    }
  }
  return new_ros_grid;
}

void SubmapPointsBatch::MapTaskLoop()
{
  while (true) {
    Task task;
    {
      std::unique_lock<std::mutex> lk(job_mx_);
      while (task_queue_.empty() && !quit_thread_) {
        tasks_condvar_.wait(lk);
      }

      if (quit_thread_) {
        return;
      }

      task = task_queue_.front();
      task_queue_.pop();
    }

    if (task.type == Task::ADD_SUBMAP) {
      {
        std::lock_guard<std::mutex> lk(display_map_mx_);
        Update(task.range_data);
      }
    } else if (task.type == Task::FLUSH) {
      // flush final map and display map with new pose
    }
  }
}

}  // namespace laser_slam
}  // namespace cartographer
