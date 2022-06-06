/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef SENSOR_RANGE_DATA_H_
#define SENSOR_RANGE_DATA_H_
#include "common/port.h"
#include "sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};



RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);
}  // namespace sensor
}  // namespace cartographer

#endif  // SENSOR_RANGE_DATA_H_
