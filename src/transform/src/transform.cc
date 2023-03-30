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
#include "transform/transform.h"

namespace cartographer {
namespace transform {
Rigid2d ToRigid2(const protos::transform::proto::Rigid2d& transform) {
  return Rigid2d({transform.translation().x(), transform.translation().y()},
                 transform.rotation());
}

Eigen::Vector2d ToEigen(const protos::transform::proto::Vector2d& vector) {
  return Eigen::Vector2d(vector.x(), vector.y());
}

Eigen::Vector3f ToEigen(const protos::transform::proto::Vector3f& vector) {
  return Eigen::Vector3f(vector.x(), vector.y(), vector.z());
}

Eigen::Vector4f ToEigen(const protos::transform::proto::Vector4f& vector) {
  return Eigen::Vector4f(vector.x(), vector.y(), vector.z(), vector.t());
}

Eigen::Vector3d ToEigen(const protos::transform::proto::Vector3d& vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Quaterniond ToEigen(
    const protos::transform::proto::Quaterniond& quaternion) {
  return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                            quaternion.z());
}

protos::transform::proto::Rigid2d ToProto(const transform::Rigid2d& transform) {
  protos::transform::proto::Rigid2d proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

protos::transform::proto::Rigid2f ToProto(const transform::Rigid2f& transform) {
  protos::transform::proto::Rigid2f proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

protos::transform::proto::Rigid3d ToProto(const transform::Rigid3d& rigid) {
  protos::transform::proto::Rigid3d proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

transform::Rigid3d ToRigid3(const protos::transform::proto::Rigid3d& rigid) {
  return transform::Rigid3d(ToEigen(rigid.translation()),
                            ToEigen(rigid.rotation()));
}

protos::transform::proto::Rigid3f ToProto(const transform::Rigid3f& rigid) {
  protos::transform::proto::Rigid3f proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

protos::transform::proto::Vector2d ToProto(const Eigen::Vector2d& vector) {
  protos::transform::proto::Vector2d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  return proto;
}

protos::transform::proto::Vector3f ToProto(const Eigen::Vector3f& vector) {
  protos::transform::proto::Vector3f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

protos::transform::proto::Vector4f ToProto(const Eigen::Vector4f& vector) {
  protos::transform::proto::Vector4f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  proto.set_t(vector.w());
  return proto;
}

protos::transform::proto::Vector3d ToProto(const Eigen::Vector3d& vector) {
  protos::transform::proto::Vector3d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

protos::transform::proto::Quaternionf ToProto(
    const Eigen::Quaternionf& quaternion) {
  protos::transform::proto::Quaternionf proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

protos::transform::proto::Quaterniond ToProto(
    const Eigen::Quaterniond& quaternion) {
  protos::transform::proto::Quaterniond proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

geometry_msgs::msg::Quaternion ToRos(const Eigen::Quaterniond& quaternion) {
  geometry_msgs::msg::Quaternion quar;
  quar.x = quaternion.x();
  quar.y = quaternion.y();
  quar.z = quaternion.z();
  quar.w = quaternion.w();
  return quar;
}

geometry_msgs::msg::Pose ToRos(const Rigid3d& rigid) {
  geometry_msgs::msg::Pose pose;
  pose.orientation = ToRos(rigid.rotation());
  pose.position.x = rigid.translation().x();
  pose.position.y = rigid.translation().y();
  pose.position.z = rigid.translation().z();
  return pose;
}

Eigen::Quaterniond FromRos(const geometry_msgs::msg::Quaternion& quarternion) {
  Eigen::Quaterniond quart;
  quart.x() = quarternion.x;
  quart.y() = quarternion.y;
  quart.z() = quarternion.z;
  quart.w() = quarternion.w;
}

Rigid3d FromRos(const geometry_msgs::msg::Pose& pose) {
  Rigid3d rigid(
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
      FromRos(pose.orientation));
  return rigid;
}

}  // namespace transform
}  // namespace cartographer