// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef LASER_SLAM__MAP_SERVER_NODE_HPP_
#define LASER_SLAM__MAP_SERVER_NODE_HPP_
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "range_data_matching/map/probability_grid_range_data_inserter_2d.h"
#include "range_data_matching/map/range_data_inserter_interface.h"
#include "range_data_matching/map/submap_2d.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

constexpr int kExpandMap = 5;
namespace cartographer
{
namespace laser_slam
{
struct PoseRangeData
{
  sensor::RangeData range_data;
  transform::Rigid3d pose;
};
class MapServerNode
{
public:
  MapServerNode()
  : publish_grid_(nullptr), quit_thread_(false), max_(-DBL_MAX, -DBL_MAX),
    min_(DBL_MAX, DBL_MAX), insertion_map_(nullptr),
    range_data_inserter_(nullptr)
  {
    ProbabilityInserterParam insert_param;
    insert_param.hit_probability = 0.55;
    insert_param.miss_probability = 0.49;
    insert_param.insert_free = true;
    range_data_inserter_ =
      absl::make_unique<mapping::ProbabilityGridRangeDataInserter2D>(
      insert_param);
  }

  virtual ~MapServerNode() {}

  void StartThread();

  void StopThread();

  void AddRangeDataAndPose(
    const sensor::RangeData & pc,
    const transform::Rigid3d & pose);

  nav_msgs::msg::OccupancyGrid grid()
  {
    nav_msgs::msg::OccupancyGrid grid;
    if (publish_grid_ != nullptr) {
      std::lock_guard<std::mutex> lk(ros_map_mtx_);
      grid = *publish_grid_;
    }
    return grid;
  }

private:
  void MapTaskThread();

  void PublishThread();

  void MapGenerateThread();

  nav_msgs::msg::OccupancyGrid::SharedPtr publish_grid_;

  std::mutex task_mtx_;
  std::mutex ros_map_mtx_;
  std::mutex insertion_map_mtx_;
  std::mutex range_data_accumulate_mtx_;
  bool quit_thread_;
  bool map_task_end_ = false;
  bool insertion_task_end_ = false;
  bool first_update_ = true;
  int num_accumulated_range_data_ = 0;
  std::shared_ptr<std::thread> map_task_thread_;
  std::shared_ptr<std::thread> map_generate_thread_;
  Eigen::Vector2d max_;
  Eigen::Vector2d min_;
  std::map<int64, geometry_msgs::msg::PoseStamped> time_pose_;
  std::deque<sensor_msgs::msg::PointCloud2> pcs_;
  std::unique_ptr<mapping::Submap2D> insertion_map_;
  std::unique_ptr<mapping::RangeDataInserterInterface> range_data_inserter_;
  mapping::ValueConversionTables convertion_tables_;
  std::deque<PoseRangeData> pose_range_datas_;
};
}  //  namespace laser_slam
}  //  namespace cartographer

#endif  //  LASER_SLAM__MAP_SERVER_NODE_HPP_
