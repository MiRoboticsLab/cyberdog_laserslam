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
#ifndef POSE_EXTRAPOLATOR_INTEGRATOR_INTEGRATION_INTERFACE_H_
#define POSE_EXTRAPOLATOR_INTEGRATOR_INTEGRATION_INTERFACE_H_
#include "sensor/imu_data.h"
#include "sensor/odometry_data.h"
// Integration with odometry factor
namespace cartographer {
namespace pose_extrapolator {
namespace integrator {
class IntergrationInterface {
 public:
  IntergrationInterface();
  virtual ~IntergrationInterface();

  virtual void AddOdometry(const sensor::OdometryData& odom_data) = 0;

  virtual void AddImuMeasurement(const sensor::ImuData& imu_data) = 0;
};
}  // namespace integrator
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTRAPOLATOR_INTEGRATOR_INTEGRATION_INTERFACE_H_
