// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef LASER_SLAM__MAP_LOADER_HPP_
#define LASER_SLAM__MAP_LOADER_HPP_
#include <memory>
#include <vector>

#include "protos/proto_stream_deserializer.h"
#include "pose_graph/bundle_adjustment.h"

namespace cartographer
{
namespace laser_slam
{
class MapLoader
{
public:
  explicit MapLoader(const stream::ProtoStreamDeserializer & deserializer);
  virtual ~MapLoader();

  const mapping::MapById<mapping::SubmapId,
    pose_graph::optimization::SubmapData> &
  submaps() const
  {
    return id_submap_data_;
  }

  const std::vector<pose_graph::optimization::Constraint> & constraints() const
  {
    return constraints_;
  }

  const mapping::MapById<mapping::NodeId,
    pose_graph::optimization::TrajectoryNode> &
  nodes() const
  {
    return id_node_data_;
  }

  const int trajectory_id() const {return trajectory_id_;}

private:
  int trajectory_id_;
  mapping::MapById<mapping::SubmapId, pose_graph::optimization::SubmapData>
  id_submap_data_;
  std::vector<pose_graph::optimization::Constraint> constraints_;
  mapping::MapById<mapping::NodeId, pose_graph::optimization::TrajectoryNode>
  id_node_data_;
  mapping::ValueConversionTables conversion_tables_;
};
typedef std::shared_ptr<MapLoader> MapLoaderPtr;
typedef std::shared_ptr<const MapLoader> MapLoaderConstPtr;
}   // namespace laser_slam
}   // namespace cartographer

#endif  // LASER_SLAM__MAP_LOADER_HPP_
