/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_EXTAPOLATOR_FILTER_HELPER_IMU_ODOM_MEASUREMENT_ASSEMBLY_H_
#define POSE_EXTAPOLATOR_FILTER_HELPER_IMU_ODOM_MEASUREMENT_ASSEMBLY_H_
#include <vector>
#include <mutex>
#include <thread>
#include <memory>

#include <eigen3/Eigen/Core>
constexpr double kNativeSensorImuBackLog = 4;  // imu back log in sec
namespace cartographer {
namespace pose_extrapolator {
struct ImuMeasurement {
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  double timestamp;  // nanosec
};
// odometry which is measurement within the time data collected
struct OdomMeasurement {
  Eigen::Vector3d position;
  Eigen::Vector3d angular;
  double timestamp;  // nanosec
  bool IsZero() {
    return position.isZero() && angular.isZero() && (timestamp == 0.0);
  }
  void SetZero() {
    position.setZero();
    angular.setZero();
    timestamp = 0.0;
  }
};
struct ImuOdomAssemblyMeasurement {
  std::vector<ImuMeasurement> imu_measurements;
  OdomMeasurement odom_measurement;
  void SetZero() {
    imu_measurements.clear();
    odom_measurement.SetZero();
    timestamp = 0.0;
  }
  double timestamp;
};
class ImuOdomMeasurementAssembly {
 public:
  ImuOdomMeasurementAssembly() {}
  virtual ~ImuOdomMeasurementAssembly() {}

  void CollectImuMeasurement(const ImuMeasurement& imu) {
    std::lock_guard<std::mutex> lk(imu_lk_);

    imu_measurements_.push_back(imu);
  }

  void GetImuOdomAssemblyData(
      const OdomMeasurement& odom,
      ImuOdomAssemblyMeasurement* assembly_measurement) {
    ImuOdomAssemblyMeasurement measurement;
    std::vector<ImuMeasurement> local_imu_measurement;
    bool imu_okay = false;
    std::lock_guard<std::mutex> lk(imu_lk_);
    {
      if (not imu_measurements_.empty()) {
        local_imu_measurement = imu_measurements_;
        EraseOlderThan(odom.timestamp, &imu_measurements_);
        imu_okay = true;
      }
    }
    if (imu_okay) {
      std::vector<ImuMeasurement>::iterator imu_iterator =
          local_imu_measurement.begin();
      for (; imu_iterator != local_imu_measurement.end(); ++imu_iterator) {
        if (imu_iterator->timestamp < odom.timestamp) {
          measurement.imu_measurements.push_back(*imu_iterator);
        } else {
          break;
        }
      }
      measurement.odom_measurement = odom;
      *assembly_measurement = measurement;
    }
  }

 private:
  template <class Container>
  void EraseOlderThan(double timestamp, Container* container) {
    typedef typename Container::value_type ValueType;
    container->erase(
        container->begin(),
        std::lower_bound(container->begin(), container->end(), timestamp,
                         [](const ValueType& value, double timestamp) -> bool {
                           return value.timestamp < timestamp;
                         }));
  }
  std::mutex imu_lk_;
  std::mutex odom_lk_;
  std::vector<ImuMeasurement> imu_measurements_;
};
typedef std::shared_ptr<ImuOdomMeasurementAssembly>
    ImuOdomMeasurementAssemblyPtr;
typedef std::shared_ptr<const ImuOdomMeasurementAssembly>
    ImuOdomMeasurementAssemblyConstPtr;
}  // namespace pose_extrapolator
}  // namespace cartographer

#endif  // POSE_EXTAPOLATOR_FILTER_HELPER_IMU_ODOM_MEASUREMENT_ASSEMBLY_H_
