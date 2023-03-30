/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RANGE_DATA_MATCHING_MAP_PROBABILITY_GRID_H_
#define RANGE_DATA_MATCHING_MAP_PROBABILITY_GRID_H_

#include <vector>

#include "common/port.h"
#include "range_data_matching/map/grid_2d.h"
#include "range_data_matching/map/map_limits.h"
#include "range_data_matching/map/xy_index.h"
#include "rviz_display/image.h"
#include "rviz_display/ros_map.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
class ProbabilityGrid : public Grid2D {
 public:
  explicit ProbabilityGrid(const MapLimits& limits,
                           ValueConversionTables* conversion_tables);

  explicit ProbabilityGrid(const protos::mapping::proto::Grid2D& proto,
                           ValueConversionTables* conversion_tables);
  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  GridType GetGridType() const override;

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const override;

  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;

  std::unique_ptr<nav_msgs::msg::OccupancyGrid> ToRosOccupancyMsg(
      double resolution, const std::string& frame_id, const rclcpp::Time& time,
      bool save_pgm, const std::string& filestem) const override;

  //   void WritePgm(const rviz_display::Image& image, const double resolution,
  //                 const std::string& filestem, bool save_pgm) override;

 private:
  uint8 ProbabilityToColor(float probability_from_grid) const;
  std::unique_ptr<rviz_display::Image> DrawProbabilityGrid(
      Eigen::Array2i* offset) const;
  ValueConversionTables* conversion_tables_;
  std::unique_ptr<rviz_display::Image> image_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_MAP_PROBABILITY_GRID_H_
