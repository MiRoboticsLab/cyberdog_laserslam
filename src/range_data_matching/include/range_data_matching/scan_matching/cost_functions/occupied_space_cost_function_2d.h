/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef RANGE_DATA_MATCHING_SCAN_MATCHING_COST_FUNCTIONS_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define RANGE_DATA_MATCHING_SCAN_MATCHING_COST_FUNCTIONS_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#include <ceres/ceres.h>

#include "range_data_matching/map/grid_2d.h"
#include "sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_SCAN_MATCHING_COST_FUNCTIONS_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
