/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef LASER_SLAM_FINAL_MAP_GENERATOR_H_
#define LASER_SLAM_FINAL_MAP_GENERATOR_H_
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.h>

#include "range_data_matching/map/grid_interface.h"
#include "range_data_matching/map/grid_2d.h"
#include "range_data_matching/map/probability_grid_range_data_inserter_2d.h"

namespace cartographer {
namespace laser_slam {
class FinalMapGenerator {
 public:
  FinalMapGenerator(const ProbabilityInserterParam& insert_param) {
    grid_ = std::make_unique<mapping::ProbabilityGrid>(
        mapping::MapLimits(0.05,
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
  nav_msgs::msg::OccupancyGrid Flush(double resolution,
                                     const std::string& frame_id,
                                     const rclcpp::Time& time, bool save_pgm,
                                     const std::string& filestem) const;

  void InsertRangeData(const sensor::RangeData& range_data);

 private:
  std::unique_ptr<mapping::RangeDataInserterInterface> grid_inserter_;
  std::unique_ptr<mapping::ProbabilityGrid> grid_;
  mapping::ValueConversionTables conversion_tables_;
};
typedef std::shared_ptr<FinalMapGenerator> FinalMapGeneratorPtr;
typedef std::shared_ptr<const FinalMapGenerator> FinalMapGeneratorConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_FINAL_MAP_GENERATOR_H_
