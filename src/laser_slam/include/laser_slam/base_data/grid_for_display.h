/**
 * @file grid_for_display.h
 * @author feixiang zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2023-01-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "range_data_matching/map/map_limits.h"
#include "range_data_matching/map/ray_to_pixel_mask.h"
#include "pose_graph/data_set/pose_graph_data.h"
#include "range_data_matching/map/probability_grid_range_data_inserter_2d.h"

constexpr int kInitMapCellsX = 100;
constexpr int kInitMapCellsY = 100;
constexpr int kSubpixelScale = 1000;
constexpr int kMapInitSize = 10000;
namespace cartographer {
namespace laser_slam {

struct Timer {
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  Timer() {}
  ~Timer() {}
  void Start() { start = std::chrono::high_resolution_clock::now(); }

  double GetTimerDurationSecond() {
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now() - start)
                        .count();
    return duration * 0.000001;
  }
};

class GridForDisplay {
 public:
  GridForDisplay()
      : first_update_(false),
        quit_thread_(false),
        is_over_size_(false),
        map_limit_(0.05, Eigen::Vector2d(0.0, 0.0),
                   mapping::CellLimits(kInitMapCellsX, kInitMapCellsY)),
        min_(DBL_MAX, DBL_MAX),
        max_(-DBL_MAX, -DBL_MAX),
        thread_(nullptr),
        publisher_(nullptr) {
    map_.header.frame_id = "laser_odom";
    map_.info.width = 2 * kMapInitSize;
    map_.info.height = 2 * kMapInitSize;
    map_.info.resolution = 0.05;
    map_.info.origin.position.x = -kMapInitSize * 0.05;
    map_.info.origin.position.y = -kMapInitSize * 0.05;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.data.resize(4 * kMapInitSize * kMapInitSize);
    for (int k = 0; k < map_.data.size(); ++k) {
      map_.data[k] = -1;
    }
    submaps_.clear();
  }
  virtual ~GridForDisplay() {}

  bool StartThread();

  bool QuitThread();

  void SetMapPublisher(const rclcpp_lifecycle::LifecyclePublisher<
                       nav_msgs::msg::OccupancyGrid>::SharedPtr& publisher) {
    publisher_ = publisher;
  }

  void AddSubmap(const mapping::SubmapId& id,
                 const std::shared_ptr<const mapping::Submap>& submap);

 private:
  void CropMapLimit();

  void MapTaskLoop();

  void StickSubmapToMap(const nav_msgs::msg::OccupancyGrid& submap);

  bool first_update_;
  bool quit_thread_;
  bool is_over_size_;
  std::mutex task_mtx_;
  std::mutex map_data_mtx_;
  mapping::MapLimits map_limit_;
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;
  nav_msgs::msg::OccupancyGrid final_map_;
  nav_msgs::msg::OccupancyGrid map_;
  std::shared_ptr<std::thread> thread_;
  std::deque<
      std::pair<mapping::SubmapId, std::shared_ptr<const mapping::Submap>>>
      submaps_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      publisher_;
};
typedef std::shared_ptr<GridForDisplay> GridForDisplayPtr;
}  // namespace laser_slam
}  // namespace cartographer
