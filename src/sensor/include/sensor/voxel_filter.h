/*
 * Copyright 2017 The Cartographer Authors
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
