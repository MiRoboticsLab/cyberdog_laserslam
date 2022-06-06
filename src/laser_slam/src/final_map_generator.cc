/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "laser_slam/final_map_generator.h"

namespace cartographer {
namespace laser_slam {
nav_msgs::msg::OccupancyGrid FinalMapGenerator::Flush(
    double resolution, const std::string& frame_id, const rclcpp::Time& time,
    bool save_pgm, const std::string& filestem) const {
  return *grid_->ToRosOccupancyMsg(resolution, frame_id, time, save_pgm,
                                   filestem);
}
void FinalMapGenerator::InsertRangeData(const sensor::RangeData& range_data) {
  grid_inserter_->Insert(range_data, grid_.get());
}

}  // namespace laser_slam
}  // namespace cartographer
