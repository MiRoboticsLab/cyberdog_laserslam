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
#include "range_data_matching/map/probability_grid.h"

#include <limits>

#include <absl/memory/memory.h>

#include "range_data_matching/map/probability_values.h"

namespace cartographer {
namespace mapping {

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

ProbabilityGrid::ProbabilityGrid(const protos::mapping::proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(proto, conversion_tables), conversion_tables_(conversion_tables) {}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid>
ProbabilityGrid::ToRosOccupancyMsg(double resolution,
                                   const std::string& frame_id,
                                   const rclcpp::Time& time, bool save_pgm,
                                   const std::string& filestem) const {
  Eigen::Array2i offset;
  auto occupancy_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  std::unique_ptr<rviz_display::Image> image = DrawProbabilityGrid(&offset);
  if (image != nullptr) {
    image->Rotate90DegreeClockWise();
    const Eigen::Vector2d origin(
        limits().max().x() -
            (offset.y() + image->width()) * limits().resolution(),
        limits().max().y() -
            (offset.x() + image->height()) * limits().resolution());
    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info.map_load_time = time;
    occupancy_grid->info.resolution = resolution;
    occupancy_grid->info.width = image->width();
    occupancy_grid->info.height = image->height();
    occupancy_grid->info.origin.position.x = origin.x();
    occupancy_grid->info.origin.position.y = origin.y();
    occupancy_grid->info.origin.position.z = 0.;
    occupancy_grid->info.origin.orientation.w = 1.;
    occupancy_grid->info.origin.orientation.x = 0.;
    occupancy_grid->info.origin.orientation.y = 0.;
    occupancy_grid->info.origin.orientation.z = 0.;
    occupancy_grid->data.reserve(image->width() * image->height());
    for (int y = image->height() - 1; y >= 0; --y) {
      for (int x = 0; x < image->width(); ++x) {
        const uint32_t packed = image->pixel_data()[y * image->width() + x];
        const uint32 color = packed;
        // const unsigned char observed = packed >> 8;
        int v;
        if (color == 128) {
          v = -1;
        } else {
          if (color > 128) {
            v = 0;
          } else {
            v = 100;
          }
        }
        const int value =
            color == 128 ? -1 : common::RoundToInt((1. - color / 255.) * 100.);
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        occupancy_grid->data.push_back(v);
      }
    }
    if (save_pgm) {
      rviz_display::StreamFileWriter pgm_writer(filestem + ".pgm");
      rviz_display::WritePgm(*image, resolution, &pgm_writer);
      Eigen::Vector2d origin = Eigen::Vector2d::Zero();
      rviz_display::StreamFileWriter yaml_writer(filestem + ".yaml");
      rviz_display::WriteYaml(resolution, origin, pgm_writer.GetFilename(),
                              &yaml_writer);
    }
  }
  return occupancy_grid;
}

// void ProbabilityGrid::WritePgm(const rviz_display::Image& image,
//                                const double resolution,
//                                const std::string& filestem) {
//   rviz_display::StreamFileWriter pgm_writer(filestem + ".pgm");
//   rviz_display::WritePgm(image_, resolution, &pgm_writer);
// }

GridType ProbabilityGrid::GetGridType() const {
  return GridType::PROBABILITY_GRID;
}

// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

uint8 ProbabilityGrid::ProbabilityToColor(float probability_from_grid) const {
  const float probability = 1.f - probability_from_grid;
  return common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}

std::unique_ptr<rviz_display::Image> ProbabilityGrid::DrawProbabilityGrid(
    Eigen::Array2i* offset) const {
  CellLimits cell_limits;
  ComputeCroppedLimits(offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Empty Probability Grid";
    return nullptr;
  }
  auto image = std::make_unique<rviz_display::Image>(cell_limits.num_x_cells,
                                                     cell_limits.num_y_cells);
  for (const Eigen::Array2i& xy_index :
       mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i index = xy_index + *offset;
    constexpr uint8 kUnknownValue = 128;
    const uint8 value = IsKnown(index)
                            ? ProbabilityToColor(GetProbability(index))
                            : kUnknownValue;
    image->SetPixel(xy_index.x(), xy_index.y(), value);
  }
  return image;
}

}  // namespace mapping
}  // namespace cartographer
