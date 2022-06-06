/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef RANGE_DATA_MATCHING_MAP_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define RANGE_DATA_MATCHING_MAP_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "common/port.h"
#include "common/param.h"
#include "range_data_matching/map/probability_grid.h"
#include "range_data_matching/map/xy_index.h"
#include "range_data_matching/map/range_data_inserter_interface.h"
#include "sensor/point_cloud.h"
#include "sensor/range_data.h"
#include "range_data_matching/map/xy_index.h"
#include "range_data_matching/map/ray_to_pixel_mask.h"
#include "range_data_matching/map/probability_values.h"

namespace cartographer {
namespace mapping {

class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
 public:
  ProbabilityGridRangeDataInserter2D(const ProbabilityInserterParam& param)
      : param_(param),
        hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
            Odds(param.hit_probability))),
        miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
            Odds(param.miss_probability))) {}
  ProbabilityGridRangeDataInserter2D(
      const ProbabilityGridRangeDataInserter2D&) = delete;
  ProbabilityGridRangeDataInserter2D& operator=(
      const ProbabilityGridRangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const override;

 private:
  ProbabilityInserterParam param_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_MAP_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
