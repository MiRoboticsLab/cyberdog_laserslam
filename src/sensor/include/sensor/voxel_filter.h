/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef SENSOR_VOXEL_FILTER_H_
#define SENSOR_VOXEL_FILTER_H_
#include <bitset>

#include "sensor/point_cloud.h"

namespace cartographer {
namespace sensor {
struct AdaptiveVoxelFilterParam {
  float min_num_points;
  float max_length;
  float max_range;
};
PointCloud VoxelFilter(const PointCloud& points, const double resolution);

PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
                               const AdaptiveVoxelFilterParam& param);

}  // namespace sensor
}  // namespace cartographer

#endif  // SENSOR_VOXEL_FILTER_H_
