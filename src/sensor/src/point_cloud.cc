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

#include "sensor/point_cloud.h"

#include "transform/transform.h"

#include <chrono>

namespace cartographer {
namespace sensor {

PointCloud::PointCloud() {}
PointCloud::PointCloud(std::vector<PointCloud::PointType> points)
    : points_(std::move(points)) {}
PointCloud::PointCloud(common::Time time, std::vector<PointType> points)
    : time_(time), points_(std::move(points)) {}
PointCloud::PointCloud(std::vector<PointType> points,
                       std::vector<float> intensities)
    : points_(std::move(points)), intensities_(std::move(intensities)) {
  if (!intensities_.empty()) {
    CHECK_EQ(points_.size(), intensities_.size());
  }
}

PointCloud::PointCloud(const protos::sensor::proto::PointCloud& proto) {
  int size = proto.point_data().size();
  points_.reserve(size);
  for (auto range_finder_point : proto.point_data()) {
    RangefinderPoint pt{transform::ToEigen(range_finder_point.position())};
    points_.push_back(pt);
  }
  time_ = common::FromUniversal(proto.timestamp());
}

protos::sensor::proto::PointCloud PointCloud::ToProto() const {
  protos::sensor::proto::PointCloud pc_proto;
  pc_proto.mutable_point_data()->Reserve(points_.size());
  for (auto& pt : points_) {
    protos::sensor::proto::RangefinderPoint pt_proto;
    *pt_proto.mutable_position() = transform::ToProto(pt.position);
    *pc_proto.add_point_data() = pt_proto;
  }
  pc_proto.set_timestamp(common::ToUniversal(time_));
  return pc_proto;
}

size_t PointCloud::size() const { return points_.size(); }
bool PointCloud::empty() const { return points_.empty(); }

const std::vector<PointCloud::PointType>& PointCloud::points() const {
  return points_;
}
const std::vector<float>& PointCloud::intensities() const {
  return intensities_;
}
const PointCloud::PointType& PointCloud::operator[](const size_t index) const {
  return points_[index];
}

PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }
PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

void PointCloud::push_back(PointCloud::PointType value) {
  points_.push_back(std::move(value));
}

sensor_msgs::msg::PointCloud2 PointCloud::ToPointCloud2() const {
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  pcl_pc.points.resize(points_.size());
  for (size_t j = 0; j < points_.size(); ++j) {
    pcl_pc.points[j].x = points_[j].position.x();
    pcl_pc.points[j].y = points_[j].position.y();
    pcl_pc.points[j].z = points_[j].position.z();
  }
  sensor_msgs::msg::PointCloud2 result_pc;
  pcl::toROSMsg(pcl_pc, result_pc);
  result_pc.header.stamp = common::ToRosTime(time_);
  return result_pc;
}

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  std::vector<RangefinderPoint> points;
  points.reserve(point_cloud.size());
  for (const RangefinderPoint& point : point_cloud.points()) {
    points.emplace_back(transform * point);
  }
  return PointCloud(points, point_cloud.intensities());
}

TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform) {
  TimedPointCloud result;
  result.reserve(point_cloud.size());
  for (const TimedRangefinderPoint& point : point_cloud) {
    result.push_back(transform * point);
  }
  return result;
}

PointCloud CropPointCloud(const PointCloud& point_cloud, const float min_z,
                          const float max_z) {
  return point_cloud.copy_if([min_z, max_z](const RangefinderPoint& point) {
    return min_z <= point.position.z() && point.position.z() <= max_z;
  });
}

}  // namespace sensor
}  // namespace cartographer
