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

#ifndef LASER_SLAM__BASE_DATA__GRID_FOR_NAVIGATION_HPP_
#define LASER_SLAM__BASE_DATA__GRID_FOR_NAVIGATION_HPP_
#include <eigen3/Eigen/Core>

#include <memory>
#include <vector>
#include <string>

#include "sensor/range_data.h"
#include "rviz_display/ros_map.h"
#include "range_data_matching/map/probability_grid_range_data_inserter_2d.h"
constexpr int64_t kKnownCell = 1;
constexpr int64_t kMissCell = 0;
constexpr int64_t kUnknowCell = -1;
constexpr int64_t kResolution = 0.05;  // m
constexpr int kPaddling = 5;
/**
 * cell coordinate perform like image:
 * -----------------------> x
 * |            width
 * |
 * |
 * | height
 * |
 * y
 *
 * the origin saved in yaml is the left top point show as above in world
 * coordinate
 *
 */
namespace cartographer
{
namespace laser_slam
{
class GridForNavigation
{
public:
  GridForNavigation(double resolution, const ProbabilityInserterParam & param)
  : width_(0.0),
    height_(0.0),
    resolution_(resolution),
    origin_(Eigen::Vector2d::Zero()),
    grid_(nullptr)
  {
    max_ << DBL_MIN, DBL_MIN;
    min_ << DBL_MAX, DBL_MAX;
    range_data_inserter_ =
      std::make_unique<mapping::ProbabilityGridRangeDataInserter2D>(param);
  }
  virtual ~GridForNavigation() {}
  void RayCast(const std::vector<sensor::RangeData> & range_datas);
  Eigen::Vector2d origin() const {return origin_;}

  bool RayCastByProbability(const std::vector<sensor::RangeData> & range_datas);

  void WritePgm(const std::string & filestem);

  void WritePgmByProbabilityGrid(const std::string & filestem);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // NO LINT

private:
  void GrowMapIfNeed(double x, double y);

  Eigen::Array2i GetCellIndex(const Eigen::Vector2d & position);

  void CastRayOnMap(const std::vector<sensor::RangeData> & range_datas);

  int width_;
  int height_;
  double resolution_;
  Eigen::Vector2d origin_;
  Eigen::Vector2d max_;
  Eigen::Vector2d min_;
  std::vector<int64> cells_;
  std::unique_ptr<mapping::RangeDataInserterInterface> range_data_inserter_;
  mapping::ValueConversionTables conversion_tables_;
  std::unique_ptr<mapping::ProbabilityGrid> grid_;
};
typedef std::shared_ptr<GridForNavigation> GridForNavigationPtr;
typedef std::shared_ptr<const GridForNavigation> GridForNavigationConstPtr;
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM__BASE_DATA__GRID_FOR_NAVIGATION_HPP_
