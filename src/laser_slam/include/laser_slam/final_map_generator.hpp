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

#ifndef LASER_SLAM__FINAL_MAP_GENERATOR_HPP_
#define LASER_SLAM__FINAL_MAP_GENERATOR_HPP_
#include <nav_msgs/msg/occupancy_grid.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "range_data_matching/map/grid_interface.h"
#include "range_data_matching/map/grid_2d.h"
#include "range_data_matching/map/probability_grid_range_data_inserter_2d.h"

namespace cartographer
{
namespace laser_slam
{
class FinalMapGenerator
{
public:
  explicit FinalMapGenerator(const ProbabilityInserterParam & insert_param)
  {
    grid_ = std::make_unique<mapping::ProbabilityGrid>(
      mapping::MapLimits(
        0.05,
        Eigen::Vector2d::Zero() +
        0.5 * 100 * 0.05 * Eigen::Vector2d::Ones(),
        mapping::CellLimits(100, 100)),
      &conversion_tables_);
    grid_inserter_ =
      std::make_unique<mapping::ProbabilityGridRangeDataInserter2D>(
      insert_param);
  }
  virtual ~FinalMapGenerator() {}
  // flush the final map when slam process end
  nav_msgs::msg::OccupancyGrid Flush(
    double resolution,
    const std::string & frame_id,
    const rclcpp::Time & time, bool save_pgm,
    const std::string & filestem) const;

  void InsertRangeData(const sensor::RangeData & range_data);

private:
  std::unique_ptr<mapping::RangeDataInserterInterface> grid_inserter_;
  std::unique_ptr<mapping::ProbabilityGrid> grid_;
  mapping::ValueConversionTables conversion_tables_;
};
typedef std::shared_ptr<FinalMapGenerator> FinalMapGeneratorPtr;
typedef std::shared_ptr<const FinalMapGenerator> FinalMapGeneratorConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM__FINAL_MAP_GENERATOR_HPP_
