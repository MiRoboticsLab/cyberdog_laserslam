/**
 * @file submap_points_batch.cc
 * @author feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-07-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "laser_slam/base_data/submap_points_batch.h"

namespace cartographer {
namespace laser_slam {
SubmapPointsBatch::SubmapPointsBatch(double resolution, int init_width,
                                     int init_height)
    : resolution_(resolution),
      quit_thread_(false),
      origin_(DBL_MAX, DBL_MAX),
      right_up_corner_(-DBL_MAX, -DBL_MAX),
      thread_(nullptr) {
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
  for (int i = 0; i < init_height * init_width; ++i) {
    final_map_.data[i] = -1;
  }
}
SubmapPointsBatch::~SubmapPointsBatch() {}

bool SubmapPointsBatch::StartThread() {
  if (thread_) {
    LOG(ERROR) << "Realtime Mapping Thread Already exist";
    return false;
  }
  thread_.reset(new std::thread(&SubmapPointsBatch::MapTaskLoop, this));
  return true;
}

bool SubmapPointsBatch::QuitThread() {
  if (thread_ != nullptr) {
    quit_thread_ = true;
    tasks_condvar_.notify_all();
    if (thread_->joinable()) thread_->join();
    thread_.reset();
  }
  return true;
}

void SubmapPointsBatch::AddSubmap(
    const mapping::SubmapId& id,
    const std::shared_ptr<const mapping::Submap>& submap) {
  std::unique_lock<std::mutex> lk(job_mx_);
  task_queue_.push(Task(id, submap));
  tasks_condvar_.notify_all();
}

void SubmapPointsBatch::Flush(
    const mapping::MapById<mapping::SubmapId,
                           pose_graph::optimization::SubmapSpec2D>&
        pose_graph_submap) {
  std::unique_lock<std::mutex> lk(job_mx_);
  task_queue_.push(Task(pose_graph_submap));
  tasks_condvar_.notify_all();
}

void SubmapPointsBatch::UpdateMap(
    const nav_msgs::msg::OccupancyGrid& new_submap) {
  Eigen::Vector2d origin(new_submap.info.origin.position.x,
                         new_submap.info.origin.position.y);
  origin_.x() = origin_.x() > origin.x() ? origin.x() : origin_.x();
  origin_.y() = origin_.y() > origin.y() ? origin.y() : origin_.y();

  int width = new_submap.info.width;
  int height = new_submap.info.height;
  Eigen::Vector2d right_corner(
      origin.x() + width * new_submap.info.resolution,
      origin.y() + height * new_submap.info.resolution);
  right_up_corner_.x() = right_corner.x() > right_up_corner_.x()
                             ? right_corner.x()
                             : right_up_corner_.x();
  right_up_corner_.y() = right_corner.y() > right_up_corner_.y()
                             ? right_corner.y()
                             : right_up_corner_.y();
  int index_x = std::ceil((origin.x() - final_map_.info.origin.position.x) /
                          final_map_.info.resolution) -
                1;
  int index_y = std::ceil((origin.y() - final_map_.info.origin.position.y) /
                          final_map_.info.resolution) -
                1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (final_map_
              .data[(y + index_y) * final_map_.info.width + x + index_x] == -1)
        final_map_.data[(y + index_y) * final_map_.info.width + x + index_x] =
            new_submap.data[y * width + x];
    }
  }
}

nav_msgs::msg::OccupancyGrid SubmapPointsBatch::CropGrid() {
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
  int index_x = std::ceil((origin_.x() - final_map_.info.origin.position.x) /
                          resolution_) -
                1;
  int index_y = std::ceil((origin_.y() - final_map_.info.origin.position.y) /
                          resolution_) -
                1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      new_ros_grid.data[y * width + x] =
          final_map_.data[(y + index_y) * final_map_.info.width + x + index_x];
    }
  }
  return new_ros_grid;
}

void SubmapPointsBatch::MapTaskLoop() {
  while (true) {
    Task task;
    {
      std::unique_lock<std::mutex> lk(job_mx_);
      while (task_queue_.empty() && !quit_thread_) tasks_condvar_.wait(lk);

      if (quit_thread_) return;

      task = task_queue_.front();
      task_queue_.pop();
    }

    if (task.type == Task::ADD_SUBMAP) {
      // Add submap and update final_map_ and display_map_
      auto ros_map = task.submap->grid()->ToRosOccupancyMsg(
          resolution_, "", rclcpp::Time::max(), false, "");
      id_rosmap_.Insert(task.id, *ros_map);
      UpdateMap(*ros_map);
      {
        std::unique_lock<std::mutex> lk(display_map_mx_);
        map_display_ = CropGrid();
      }
    } else if (task.type == Task::FLUSH) {
      // flush final map and display map with new pose
    }
  }
}

}  // namespace laser_slam
}  // namespace cartographer
