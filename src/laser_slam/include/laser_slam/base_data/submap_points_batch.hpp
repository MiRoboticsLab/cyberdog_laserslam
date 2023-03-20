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

#ifndef LASER_SLAM__BASE_DATA__SUBMAP_POINTS_BATCH_HPP_
#define LASER_SLAM__BASE_DATA__SUBMAP_POINTS_BATCH_HPP_
#include <eigen3/Eigen/Core>

#include <condition_variable>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>

#include "transform/rigid_transform.h"
#include "range_data_matching/map/id.h"
#include "range_data_matching/map/submap_2d.h"
#include "pose_graph/data_set/pose_graph_data.h"

namespace cartographer
{
namespace laser_slam
{
struct Task
{
  enum Type { ADD_SUBMAP, FLUSH, EXTEND_MAP } type;
  mapping::SubmapId id;
  std::shared_ptr<const mapping::Submap> submap;
  sensor::RangeData range_data;
  mapping::NodeId node_id;
  mapping::MapById<mapping::SubmapId, pose_graph::optimization::SubmapSpec2D>
  flushed_pose;
  Task()
  : id{0, 0}, node_id{0, 0} {}
  Task(
    const mapping::SubmapId & s_id,
    const std::shared_ptr<const mapping::Submap> & s_submap)
  : type(ADD_SUBMAP), id(s_id), node_id{0, 0}, submap(s_submap) {}

  Task(const mapping::NodeId & node_id, const sensor::RangeData & range_data)
  : type(ADD_SUBMAP), id{0, 0}, node_id(node_id), range_data(range_data) {}
  Task(
    const mapping::MapById<mapping::SubmapId,
    pose_graph::optimization::SubmapSpec2D> & poses)
  : type(FLUSH), id{0, 0}, node_id{0, 0}, flushed_pose(poses) {}
};
class SubmapPointsBatch
{
public:
  typedef std::queue<Task> TaskQueue;
  SubmapPointsBatch(double resolution, int init_width, int init_height);
  virtual ~SubmapPointsBatch();

  bool StartThread();

  bool QuitThread();

  void AddSubmap(
    const mapping::SubmapId & id,
    const std::shared_ptr<const mapping::Submap> & submap);

  void AddRangeData(
    const mapping::NodeId & id,
    const sensor::RangeData & range_data);

  void Flush(
    const mapping::MapById<mapping::SubmapId,
    pose_graph::optimization::SubmapSpec2D> &
    pose_graph_submap);
  nav_msgs::msg::OccupancyGrid ros_grid()
  {
    std::unique_lock<std::mutex> lk(display_map_mx_);
    return final_map_;
  }

  bool is_grid() {return is_grid_;}

private:
  // loop for mapping realtime
  void MapTaskLoop();
  // update map every time submap coming
  void UpdateMap(const nav_msgs::msg::OccupancyGrid & new_submap);

  // Update map every time range data comming
  void Update(const sensor::RangeData & range_data);
  // crop the map size suit for map now
  nav_msgs::msg::OccupancyGrid CropGrid();

  double resolution_;
  bool quit_thread_;
  bool first_update_ = true;
  bool is_grid_ = false;
  int submap_count_;
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

#endif  // LASER_SLAM__BASE_DATA__SUBMAP_POINTS_BATCH_HPP_
