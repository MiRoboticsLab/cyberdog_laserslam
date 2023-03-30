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
#ifndef RVIZ_DISPLAY_PROBABILITY_GRID_TO_IMAGE_H_
#define RVIZ_DISPLAY_PROBABILITY_GRID_TO_IMAGE_H_
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "common/port.h"
#include "range_data_matching/map/probability_grid.h"
#include "range_data_matching/map/value_conversion_tables.h"
#include "rviz_display/image.h"

namespace cartographer {
namespace rviz_display {
class ProbabilityGridConversion {
 public:
  ProbabilityGridConversion(const std::string& topic_name) {}
  virtual ~ProbabilityGridConversion() {}

  void CreateOccupiedGridAndPublish(const mapping::Grid2D* grid,
                                    double resolution,
                                    const std::string& frame_id,
                                    const rclcpp::Time& time) {
    auto occupied_grid = CreateOccupiedGrid(grid, resolution, frame_id, time);
  }

 private:
  std::unique_ptr<Image> DrawProbabilityGrid(const mapping::Grid2D* grid,
                                             Eigen::Array2i* offset);

  uint8 ProbabilityToColor(float probability_from_grid);

  std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupiedGrid(
      const mapping::Grid2D* grid, double resolution,
      const std::string& frame_id, const rclcpp::Time& time);
};
}  // namespace rviz_display
}  // namespace cartographer

#endif  // RVIZ_DISPLAY_PROBABILITY_GRID_TO_IMAGE_H_
