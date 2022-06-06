/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "rviz_display/probability_grid_conversion.h"

namespace cartographer {
namespace rviz_display {
std::unique_ptr<nav_msgs::msg::OccupancyGrid>
ProbabilityGridConversion::CreateOccupiedGrid(const mapping::Grid2D* grid,
                                              double resolution,
                                              const std::string& frame_id,
                                              const rclcpp::Time& time) {
  Eigen::Array2i offset;
  auto occupancy_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  std::unique_ptr<Image> image = DrawProbabilityGrid(grid, &offset);
  if (image != nullptr) {
    const auto& limits = grid->limits();
    image->Rotate90DegreeClockWise();
    const Eigen::Vector2d origin(
        limits.max().x() - (offset.y() + image->width()) * limits.resolution(),
        limits.max().y() -
            (offset.x() + image->height()) * limits.resolution());
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
        const unsigned char color = packed >> 16;
        const unsigned char observed = packed >> 8;
        const int value =
            observed == 0 ? -1 : common::RoundToInt((1. - color / 255.) * 100.);
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        occupancy_grid->data.push_back(value);
      }
    }
  }
  return occupancy_grid;
}

uint8 ProbabilityGridConversion::ProbabilityToColor(
    float probability_from_grid) {
  const float probability = 1.f - probability_from_grid;
  return common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}

std::unique_ptr<Image> ProbabilityGridConversion::DrawProbabilityGrid(
    const mapping::Grid2D* grid, Eigen::Array2i* offset) {
  mapping::CellLimits cell_limits;
  grid->ComputeCroppedLimits(offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Empty Probability Grid";
    return nullptr;
  }
  auto image =
      std::make_unique<Image>(cell_limits.num_x_cells, cell_limits.num_y_cells);
  for (const Eigen::Array2i& xy_index :
       mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i index = xy_index + *offset;
    constexpr uint8 kUnknownValue = 128;
    LOG(INFO) << "start to convert";
    const uint8 value = grid->IsKnown(index)
                            ? ProbabilityToColor(grid->GetProbability(index))
                            : kUnknownValue;
    image->SetPixel(xy_index.x(), xy_index.y(), {{value, value, value}});
  }
  return image;
}

}  // namespace rviz_display
}  // namespace cartographer