/**
 * @file real_time_map.h
 * @author feixiang zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-12-26
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
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

namespace cartographer {
namespace laser_slam {
struct Task {
  mapping::NodeId id;
  sensor::RangeData range_data;
  transform::Rigid3d origin;

  Task(const mapping::NodeId& id, const sensor::RangeData& range_data,
       const transform::Rigid3d& origin)
      : id(id), range_data(range_data), origin(origin) {}
};

}  // namespace laser_slam
}  // namespace cartographer
