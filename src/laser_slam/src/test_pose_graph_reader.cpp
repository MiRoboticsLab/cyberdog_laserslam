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
#include <string>
#include <vector>

#include "laser_slam/map_loader.hpp"
#include "laser_slam/base_data/grid_for_navigation.hpp"

#include "protos/proto_stream_interface.h"
#include "protos/proto_stream.h"
#include "protos/proto_stream_deserializer.h"
#include "range_data_matching/map/grid_2d.h"
#include "range_data_matching/map/probability_grid.h"

int main(int argc, char ** argv)
{
  std::string file_path = "/home/zfx/map_test/graph.pbstream";
  cartographer::stream::ProtoStreamReader reader(file_path);
  cartographer::stream::ProtoStreamDeserializer deserializer(&reader);
  protos::mapping::proto::PoseGraph proto;
  cartographer::ProbabilityInserterParam param;
  param.hit_probability = 0.55;
  param.insert_free = true;
  param.miss_probability = 0.49;
  cartographer::laser_slam::GridForNavigation grid_inserter(0.05, param);
  
  LOG(INFO) << deserializer.pose_graph().constraint().size() << " , " <<
    deserializer.pose_graph().submaps().size() << " , " <<
    deserializer.pose_graph().submaps(3).global_pose().DebugString() <<
    " , " <<
    deserializer.pose_graph().submaps(3).submap().grid().cells(1345);
  cartographer::mapping::ValueConversionTables table;
  cartographer::mapping::ProbabilityGrid grid(
    deserializer.pose_graph().submaps(3).submap().grid(), &table);
  cartographer::laser_slam::MapLoader loader(deserializer);
  LOG(INFO) << "load constraint size is: " << loader.constraints().size() <<
    "load node is: " << loader.nodes().size() <<
    "load submap is: " << loader.submaps().size();
  LOG(INFO) << " max is: " << grid.limits().max().transpose();
  std::vector<cartographer::sensor::RangeData> range_datas;
  for (auto pc : loader.nodes()) {
    cartographer::sensor::PointCloud p =
      pc.data.constant_data->filtered_gravity_aligned_point_cloud;
    p = cartographer::sensor::TransformPointCloud(
      p, pc.data.constant_data->local_pose.cast<float>());
    cartographer::sensor::RangeData range;
    range.returns = p;
    range.origin = pc.data.global_pose.translation().cast<float>();
    range_datas.push_back(range);
  }
  grid_inserter.RayCastByProbability(range_datas);
  grid_inserter.WritePgmByProbabilityGrid("/home/zfx/test_load");
  for (auto id_submap : loader.submaps()) {
    LOG(INFO) << id_submap.id.submap_index << " pose is: " <<
      id_submap.data.submap->local_pose().DebugString() <<
      "submap : " << id_submap.data.submap->num_range_data();
  }
  // coordinate is x right y down z forward
  Eigen::Matrix4d odom_t_camera;
  odom_t_camera << 0.00136008, -0.99997964, 0.00623416, -0.32238073, 0.00479473,
    -0.00622757, -0.99996911, 0.0126453, 0.99998758, 0.00138993, 0.00478617,
    0.0369314, 0., 0., 0., 1.;

  Eigen::Matrix4d transform;
  transform << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
  auto new_odom_t_camera = odom_t_camera * transform;
  LOG(INFO) << new_odom_t_camera;

  LOG(INFO) << new_odom_t_camera;

  LOG(INFO) << "camera to odom : " << new_odom_t_camera.inverse();

  // coordinate is x left y backward z up
  Eigen::Matrix4d camera_t_laser;
  camera_t_laser << -0.1086543842861609, -0.9940563429529429,
    0.00679792692048635, 0.2184301238445504, 0.01024554739395747,
    -0.007957877391885804, -0.9999158469321374, -0.0165978918789496,
    0.9940267871310076, -0.108575592203942, 0.0110493096422285,
    -0.1933048776018878, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix4d transform1;
  transform1 << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
  auto new_camera_t_laser = camera_t_laser * transform1;
  LOG(INFO) << "camera t laser is: " << new_camera_t_laser;
  LOG(INFO) << "laser t camera: " << new_camera_t_laser.inverse();
  LOG(INFO) << "laser t odom : " << new_camera_t_laser.inverse() * new_odom_t_camera.inverse();

  return 0;
}
