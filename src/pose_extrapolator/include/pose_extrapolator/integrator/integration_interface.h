/**
 * @file integration_interface.h
 * @author Feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */
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
