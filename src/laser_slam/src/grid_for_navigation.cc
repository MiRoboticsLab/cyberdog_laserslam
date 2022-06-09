/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/base_data/grid_for_navigation.h"

namespace cartographer {
namespace laser_slam {
void GridForNavigation::RayCastByProbability(
    const std::vector<sensor::RangeData>& range_datas) {
  CHECK(!range_datas.empty());
  for (size_t i = 0; i < range_datas.size(); ++i) {
    if (grid_ == nullptr) {
      Eigen::Vector2f origin = range_datas[i].origin.head(2);
      grid_ = std::make_unique<mapping::ProbabilityGrid>(
          mapping::MapLimits(
              resolution_,
              origin.cast<double>() +
                  0.5 * 100 * resolution_ * Eigen::Vector2d::Ones(),
              mapping::CellLimits(100, 100)),
          &conversion_tables_);
    }
    range_data_inserter_->Insert(range_datas[i], grid_.get());
  }
  // width_ = grid_->limits().cell_limits().num_y_cells;
  // height_ = grid_->limits().cell_limits().num_x_cells;
}

void GridForNavigation::RayCast(
    const std::vector<sensor::RangeData>& range_datas) {
  CHECK(!range_datas.empty());
  for (size_t i = 0; i < range_datas.size(); ++i) {
    GrowMapIfNeed(range_datas[i].origin.x(), range_datas[i].origin.y());
    for (size_t j = 0; j < range_datas[i].returns.size(); ++j) {
      GrowMapIfNeed(range_datas[i].returns[j].position.x(),
                    range_datas[i].returns[j].position.y());
    }
    for (size_t n = 0; n < range_datas[i].misses.size(); ++n) {
      GrowMapIfNeed(range_datas[i].misses[n].position.x(),
                    range_datas[i].misses[n].position.y());
    }
  }
  origin_ = max_;
  width_ = std::ceil(fabs(max_.x() - min_.x()) / resolution_) + kPaddling;
  height_ = std::ceil(fabs(max_.y() - min_.y()) / resolution_) + kPaddling;

  cells_ = std::vector<int64>(width_ * height_, kUnknowCell);
  CastRayOnMap(range_datas);
}

void GridForNavigation::GrowMapIfNeed(double x, double y) {
  max_.x() = y > max_.x() ? y : max_.x();
  max_.y() = x > max_.y() ? x : max_.y();
  min_.x() = y < min_.x() ? y : min_.x();
  min_.y() = x < min_.y() ? x : min_.y();
}

Eigen::Array2i GridForNavigation::GetCellIndex(
    const Eigen::Vector2d& position) {
  int cell_x = std::ceil((max_.x() - position.y()) / resolution_);
  int cell_y = std::ceil((max_.y() - position.x()) / resolution_);
  Eigen::Array2i result_cell;
  result_cell.x() = cell_x;
  result_cell.y() = cell_y;
  return result_cell;
}

void GridForNavigation::CastRayOnMap(
    const std::vector<sensor::RangeData>& range_datas) {
  for (size_t i = 0; i < range_datas.size(); ++i) {
    for (size_t j = 0; j < range_datas[i].returns.size(); ++j) {
      Eigen::Vector2d pose(
          range_datas[i].returns[j].position.cast<double>().x(),
          range_datas[i].returns[j].position.cast<double>().y());
      Eigen::Array2i cell_index = GetCellIndex(pose);
      cells_[cell_index.y() * width_ + cell_index.x()] = kKnownCell;
      //   cells_.setConstant(cell_index.y(), cell_index.x(), kKnownCell);
    }
    for (size_t n = 0; n < range_datas[i].misses.size(); ++n) {
      Eigen::Vector2d pose_miss(
          range_datas[i].misses[n].position.cast<double>().x(),
          range_datas[i].misses[n].position.cast<double>().y());
      Eigen::Array2i cell_index_miss = GetCellIndex(pose_miss);
      cells_[cell_index_miss.y() * width_ + cell_index_miss.x()] = kMissCell;
      // cells_.setConstant(cell_index_miss.y(), cell_index_miss.x(),
      // kMissCell);
    }
  }
}

// void GridForNavigation::CastOpenArea(const Eigen::Vector3f& origin, const
// Eigen::Vector3f& hit) {

// }
void GridForNavigation::WritePgmByProbabilityGrid(const std::string& filestem) {
  Eigen::Array2i offset;
  mapping::CellLimits cell_limits;
  // auto occupied_grid = grid_->ToRosOccupancyMsg(0.05, "laser_odom",
  //                                               rclcpp::Time::max(), false,
  //                                               "");
  // LOG(INFO) << "origin is: " << occupied_grid->info.origin.position.x << " ,
  // "
  //           << occupied_grid->info.origin.position.y;
  const auto& map_limits = grid_->limits();
  grid_->ComputeCroppedLimits(&offset, &cell_limits);
  LOG(INFO) << "offset is: " << offset;
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    LOG(WARNING) << "Empty Probability Grid";
    return;
  }
  int width = cell_limits.num_x_cells;
  int height = cell_limits.num_y_cells;
  std::vector<uint32> data(width * height);
  for (const Eigen::Array2i& xy_index :
       mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i index = xy_index + offset;
    uint32 value;
    if (grid_->IsKnown(index)) {
      float probability = grid_->GetProbability(index);
      const float pb = 1.f - probability;
      value = common::RoundToInt(
          255 * ((pb - mapping::kMinProbability) /
                 (mapping::kMaxProbability - mapping::kMinProbability)));
    } else {
      value = 128;
    }
    data[xy_index.y() * width + xy_index.x()] = value;
  }
  // rotate 90 degree clockwise
  const auto old_data = data;
  data.clear();
  for (int x = 0; x < width; ++x) {
    for (int y = height - 1; y >= 0; --y) {
      data.push_back(old_data.at(y * width + x));
    }
  }
  LOG(INFO) << "width is: " << width << "heght is: " << height;
  std::swap(width, height);
  LOG(INFO) << "width is: " << width << "heght is: " << height;
  LOG(INFO) << "max is: " << map_limits.max().transpose();
  const Eigen::Vector2d origin(
      map_limits.max().x() - (offset.y() + width) * map_limits.resolution(),
      map_limits.max().y() - (offset.x() + height) * map_limits.resolution());
  const std::string header =
      "P5\n# Cartographer map; " + std::to_string(resolution_) + " m/pixel\n" +
      std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  rviz_display::StreamFileWriter pgm_writer(filestem + ".pgm");
  pgm_writer.Write(header.data(), header.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int32 color = data[y * width + x];
      const char v = color;
      pgm_writer.Write(&v, 1);
    }
  }
  CHECK(pgm_writer.Close());
  const std::string output =
      "image: " + pgm_writer.GetFilename() + "\n" +
      "resolution: " + std::to_string(resolution_) + "\n" + "origin: [" +
      std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
      ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  rviz_display::StreamFileWriter yaml_writer(filestem + ".yaml");
  yaml_writer.Write(output.data(), output.size());
  CHECK(yaml_writer.Close());
}

void GridForNavigation::WritePgm(const std::string& filestem) {
  const std::string header =
      "P5\n# Cartographer map; " + std::to_string(resolution_) + " m/pixel\n" +
      std::to_string(width_) + " " + std::to_string(height_) + "\n255\n";
  rviz_display::StreamFileWriter pgm_writer(filestem + ".pgm");
  pgm_writer.Write(header.data(), header.size());
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int64 value = cells_[y * width_ + x];
      if (value == kKnownCell) {
        constexpr uint32 color_i = 0;
        const char color = color_i;
        pgm_writer.Write(&color, 1);
      } else if (value == kUnknowCell) {
        constexpr uint32 j = 128;
        const char color = j;
        pgm_writer.Write(&color, 1);
      } else if (value == kMissCell) {
        constexpr uint32 h = 255;
        const char color = h;
        pgm_writer.Write(&color, 1);
      }
    }
  }
}
}  // namespace laser_slam
}  // namespace cartographer