/**
 * @file submap_points_batch.h
 * @author Feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef LASER_SLAM_BASE_DATA_SUBMAP_POINTS_BATCH_H_
#define LASER_SLAM_BASE_DATA_SUBMAP_POINTS_BATCH_H_
#include <condition_variable>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>

#include <eigen3/Eigen/Core>

#include "transform/rigid_transform.h"
#include "range_data_matching/map/id.h"
#include "range_data_matching/map/submap_2d.h"
#include "pose_graph/data_set/pose_graph_data.h"

namespace cartographer {
namespace laser_slam {
struct Task {
  enum Type { ADD_SUBMAP, FLUSH, EXTEND_MAP } type;
  mapping::SubmapId id;
  std::shared_ptr<const mapping::Submap> submap;
  mapping::MapById<mapping::SubmapId, pose_graph::optimization::SubmapSpec2D>
      flushed_pose;
  Task() : id{0, 0} {}
  Task(const mapping::SubmapId& s_id,
       const std::shared_ptr<const mapping::Submap>& s_submap)
      : type(ADD_SUBMAP), id(s_id), submap(s_submap) {}
  Task(const mapping::MapById<mapping::SubmapId,
                              pose_graph::optimization::SubmapSpec2D>& poses)
      : type(FLUSH), id{0, 0}, flushed_pose(poses) {}
};
class SubmapPointsBatch {
 public:
  typedef std::queue<Task> TaskQueue;
  SubmapPointsBatch(double resolution, int init_width, int init_height);
  virtual ~SubmapPointsBatch();

  bool StartThread();

  bool QuitThread();

  void AddSubmap(const mapping::SubmapId& id,
                 const std::shared_ptr<const mapping::Submap>& submap);

  void Flush(const mapping::MapById<mapping::SubmapId,
                                    pose_graph::optimization::SubmapSpec2D>&
                 pose_graph_submap);
  nav_msgs::msg::OccupancyGrid ros_grid() {
    std::unique_lock<std::mutex> lk(display_map_mx_);
    return map_display_;
  }

  bool is_grid() { return !id_rosmap_.empty(); }

 private:
  // loop for mapping realtime
  void MapTaskLoop();
  // update map every time submap coming
  void UpdateMap(const nav_msgs::msg::OccupancyGrid& new_submap);
  // crop the map size suit for map now
  nav_msgs::msg::OccupancyGrid CropGrid();

  double resolution_;
  bool quit_thread_;
  std::mutex job_mx_;
  std::mutex display_map_mx_;
  mapping::MapById<mapping::SubmapId, nav_msgs::msg::OccupancyGrid> id_rosmap_;
  nav_msgs::msg::OccupancyGrid final_map_;
  nav_msgs::msg::OccupancyGrid map_display_;
  Eigen::Vector2d origin_;
  Eigen::Vector2d right_up_corner_;
  TaskQueue task_queue_;
  std::condition_variable tasks_condvar_;
  std::shared_ptr<std::thread> thread_;
};
typedef std::shared_ptr<SubmapPointsBatch> SubmapPointsBatchPtr;
typedef std::shared_ptr<const SubmapPointsBatch> SubmapPointsBatchConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_BASE_DATA_SUBMAP_POINTS_BATCH_H_
