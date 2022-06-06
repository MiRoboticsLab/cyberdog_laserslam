/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef LASER_SLAM_MAP_LOADER_H_
#define LASER_SLAM_MAP_LOADER_H_
#include "protos/proto_stream_deserializer.h"
#include "pose_graph/bundle_adjustment.h"

namespace cartographer {
namespace laser_slam {
class MapLoader {
 public:
  MapLoader(const stream::ProtoStreamDeserializer& deserializer);
  virtual ~MapLoader();

  const mapping::MapById<mapping::SubmapId,
                         pose_graph::optimization::SubmapData>&
  submaps() const {
    return id_submap_data_;
  }

  const std::vector<pose_graph::optimization::Constraint>& constraints() const {
    return constraints_;
  }

  const mapping::MapById<mapping::NodeId,
                         pose_graph::optimization::TrajectoryNode>&
  nodes() const {
    return id_node_data_;
  }

  const int trajectory_id() const { return trajectory_id_;}

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
}  // namespace laser_slam
}  // namespace cartographer

#endif  // LASER_SLAM_MAP_LOADER_H_
