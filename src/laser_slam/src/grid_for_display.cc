/**
 * @file grid_for_display.cc
 * @author feixiang zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2023-01-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "laser_slam/base_data/grid_for_display.h"

namespace cartographer {
namespace laser_slam {
bool GridForDisplay::StartThread() {
  if (thread_) {
    LOG(ERROR) << "Realtime Mapping Thread Already exist";
    return false;
  }
  thread_.reset(new std::thread(&GridForDisplay::MapTaskLoop, this));
  return true;
}

bool GridForDisplay::QuitThread() {
  if (thread_ != nullptr) {
    quit_thread_ = true;
    if (thread_->joinable()) thread_->join();
    thread_.reset();
  }
  return true;
}

void GridForDisplay::CropMapLimit() {
  int width = (max_.x() - min_.x()) / 0.05;
  int height = (max_.y() - min_.y()) / 0.05;
  final_map_.info.height = height;
  final_map_.info.width = width;
  final_map_.info.resolution = 0.05;
  final_map_.info.origin.position.x = min_.x();
  final_map_.info.origin.position.y = min_.y();
  final_map_.info.origin.position.z = 0;
  final_map_.info.origin.orientation.w = 1.0;
  final_map_.info.origin.orientation.x = 0.0;
  final_map_.info.origin.orientation.y = 0.0;
  final_map_.info.origin.orientation.z = 0.0;
  final_map_.data.clear();
  final_map_.data.resize(width * height);
  int index_x = std::ceil((min_.x() - map_.info.origin.position.x) / 0.05) - 1;
  int index_y = std::ceil((min_.y() - map_.info.origin.position.y) / 0.05) - 1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      final_map_.data[y * width + x] =
          map_.data[(y + index_y) * map_.info.width + x + index_x];
    }
  }
}

void GridForDisplay::StickSubmapToMap(
    const nav_msgs::msg::OccupancyGrid& submap) {
  int width = submap.info.width;
  int height = submap.info.height;
  Eigen::Vector2d s_origin;
  s_origin.x() = submap.info.origin.position.x;
  s_origin.y() = submap.info.origin.position.y;
  int width_offset =
      std::ceil((s_origin.x() - map_.info.origin.position.x) / 0.05);
  int height_offset =
      std::ceil((s_origin.y() - map_.info.origin.position.y) / 0.05);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (map_.data[(y + height_offset) * map_.info.width + x + width_offset] ==
          -1) {
        map_.data[(y + height_offset) * map_.info.width + x + width_offset] =
            submap.data[y * width + x];
      }
    }
  }
}

void GridForDisplay::AddSubmap(
    const mapping::SubmapId& id,
    const std::shared_ptr<const mapping::Submap>& submap) {
  if (is_over_size_) return;
  Eigen::Array2i offset;
  mapping::CellLimits cell_limits;
  submap->grid()->ComputeCroppedLimits(&offset, &cell_limits);
  auto map_max = submap->grid()->limits().max();
  Eigen::Vector2d min;
  min.x() = map_max.x() - (cell_limits.num_y_cells + offset.y()) * 0.05;
  min.y() = map_max.y() - (cell_limits.num_x_cells + offset.x()) * 0.05;
  Eigen::Vector2d max;
  max.x() = min.x() + cell_limits.num_y_cells * 0.05;
  max.y() = min.y() + cell_limits.num_x_cells * 0.05;
  max_.x() = max.x() > max_.x() ? max.x() : max_.x();
  max_.y() = max.y() > max_.y() ? max.y() : max_.y();
  min_.x() = min.x() < min_.x() ? min.x() : min_.x();
  min_.y() = min.y() < min_.y() ? min.y() : min_.y();
  // if the map oversize, stop painting
  double map_max_x = map_.info.origin.position.x + map_.info.width * 0.05;
  double map_max_y = map_.info.origin.position.y + map_.info.height * 0.05;
  if (min_.x() < map_.info.origin.position.x ||
      min_.y() < map_.info.origin.position.y || max_.x() > map_max_x ||
      max_.y() > map_max_y) {
    is_over_size_ = true;
    return;
  }
  {
    std::lock_guard<std::mutex> lk(task_mtx_);

    if (submaps_.empty() || submaps_.back().first != id) {
      submaps_.push_back(std::make_pair(id, submap));
    } else if (submaps_.back().first == id) {
      submaps_.back() = std::make_pair(id, submap);
    }
  }
}

void GridForDisplay::MapTaskLoop() {
  Timer timer;
  while (true) {
    while (submaps_.empty() && !quit_thread_) {
      usleep(1000);
    }
    if (!first_update_) {
      timer.Start();
      first_update_ = true;
    }
    if (quit_thread_) return;
    std::shared_ptr<const mapping::Submap> map;
    {
      std::lock_guard<std::mutex> lk(task_mtx_);
      map = submaps_.front().second;
      submaps_.pop_front();
    }
    if (map->insertion_finished()) {
      auto submap_ros = map->grid()->ToRosOccupancyMsg(
          0.05, "laser_odom", rclcpp::Time::max(), false, " ");

      StickSubmapToMap(*submap_ros);
    }
    if (timer.GetTimerDurationSecond() > 1) {
      if (!map->insertion_finished()) {
        auto map_ros = map->grid()->ToRosOccupancyMsg(
            0.05, "laser_odom", rclcpp::Time::max(), false, "");
        StickSubmapToMap(*map_ros);
      }
      CropMapLimit();
      // CropMapLimit();
      final_map_.header.frame_id = "laser_odom";
      publisher_->publish(final_map_);

      timer.Start();
    }
  }
}

}  // namespace laser_slam
}  // namespace cartographer
