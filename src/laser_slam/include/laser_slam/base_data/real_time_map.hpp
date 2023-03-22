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

#ifndef LASER_SLAM__BASE_DATA__REAL_TIME_MAP_HPP_
#define LASER_SLAM__BASE_DATA__REAL_TIME_MAP_HPP_
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
  mapping::NodeId id;
  sensor::RangeData range_data;
  transform::Rigid3d origin;

  Task(
    const mapping::NodeId & id, const sensor::RangeData & range_data,
    const transform::Rigid3d & origin)
  : id(id), range_data(range_data), origin(origin) {}
};

}  // namespace laser_slam
}  // namespace cartographer
#endif  // LASER_SLAM__BASE_DATA__REAL_TIME_MAP_HPP_
