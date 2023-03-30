// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef RANGE_DATA_MATCHING_FILTER_COMPLEMENTARY_FILTER_H_
#define RANGE_DATA_MATCHING_FILTER_COMPLEMENTARY_FILTER_H_
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>

#include "range_data_matching/filter/filter_helper/math_util.h"
constexpr double kGravity = 9.81;
// Follow the work https://www.mdpi.com/107542 which get good attitude from
// complementary filter
namespace cartographer {
namespace mapping {
class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(double alpha) : alpha_(alpha) {}
  virtual ~ComplementaryFilter() {}

  void Update(const Eigen::Quaterniond& predicted_rotation,
              const Eigen::Vector3d& acc, Eigen::Quaterniond* updated_rotation);

  void GetMeasurement(const Eigen::Vector3d& acc, Eigen::Quaterniond* meas);

 private:
  double GetAdaptiveGain(double alpha, const Eigen::Vector3d& acc);
  void ScaleQuaternion(double gain, const Eigen::Quaterniond& delta_q,
                       Eigen::Quaterniond* q);

  double alpha_;
};
typedef std::shared_ptr<const ComplementaryFilter> ComplementaryFilterConstPtr;
typedef std::shared_ptr<ComplementaryFilter> ComplementaryFilterPtr;
}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_FILTER_COMPLEMENTARY_FILTER_H_
