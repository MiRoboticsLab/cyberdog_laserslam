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
#include <string>

#include "laser_slam/final_map_generator.hpp"

namespace cartographer
{
namespace laser_slam
{
nav_msgs::msg::OccupancyGrid FinalMapGenerator::Flush(
  double resolution, const std::string & frame_id, const rclcpp::Time & time,
  bool save_pgm, const std::string & filestem) const
{
  return *grid_->ToRosOccupancyMsg(
    resolution, frame_id, time, save_pgm,
    filestem);
}
void FinalMapGenerator::InsertRangeData(const sensor::RangeData & range_data)
{
  grid_inserter_->Insert(range_data, grid_.get());
}

}  // namespace laser_slam
}  // namespace cartographer
