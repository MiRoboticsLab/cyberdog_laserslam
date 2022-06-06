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

#include "range_data_matching/map/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include <absl/memory/memory.h>

#include "common/port.h"
#include "range_data_matching/map/range_data_inserter_interface.h"

namespace cartographer {
namespace mapping {

Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}

Submap2D::Submap2D(const protos::mapping::proto::Submap2D& proto,
                   ValueConversionTables* conversion_table)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_table) {
  grid_ = absl::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
  set_num_range_data(proto.num_range_data());
  set_insertion_finished(proto.finished());
}

protos::mapping::proto::Submap2D Submap2D::ToProto(
    const bool include_grid_data) const {
  protos::mapping::proto::Submap2D proto;
  *proto.mutable_local_pose() = transform::ToProto(local_pose());
  proto.set_num_range_data(num_range_data());
  proto.set_finished(insertion_finished());
  if (include_grid_data) {
    CHECK(grid_);
    *proto.mutable_grid() = grid_->ToProto();
  }
  return proto;
}

void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  CHECK(grid_);
  CHECK(!insertion_finished());
  CHECK_NOTNULL(range_data_inserter);
  range_data_inserter->Insert(range_data, grid_.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  set_insertion_finished(true);
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == param_.num_range_data) {
    AddSubmap(range_data.origin.head<2>());
  }
  LOG(INFO) << "add submap "
            << " size is: " << submaps_.size();
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  LOG(INFO) << "insert end";
  if (submaps_.front()->num_range_data() == 2 * param_.num_range_data) {
    submaps_.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = param_.resolution;
  if (param_.grid_type == kProbabilityGridType) {
    return absl::make_unique<ProbabilityGrid>(
        MapLimits(resolution,
                  origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                              resolution *
                                              Eigen::Vector2d::Ones(),
                  CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
        &conversion_tables_);
  } else {
    LOG(ERROR) << "Unknown grid type";
    return nullptr;
  }
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
