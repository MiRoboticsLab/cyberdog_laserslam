/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_PREINTEGRATION_FACTOR_ON_MAINFOLD_H_
#define IMU_INTEGRATION_PREINTEGRATION_FACTOR_ON_MAINFOLD_H_
#include <vector>

#include "imu_integration/integration_helper/so3.h"

namespace imu_integration {
struct ImuDatas {
  std::vector<Eigen::Vector3d> acc_datas;
  std::vector<Eigen::Vector3d> gyro_datas;
  std::vector<double> dts;
};

class PreintegrationFactorOnManifold {
 public:
  PreintegrationFactorOnManifold() {
    delta_p_.setZero();
    delta_v_.setZero();
    delta_rot_.setZero();
  }
  virtual ~PreintegrationFactorOnManifold() {}
  void PreIntegrate(const ImuDatas& collected_data);
  void UpdateFactor();

 private:
  // pre-integration measurements [position, velocity, rotation] between two
  // keyframes
  Eigen::Vector3d delta_p_;
  Eigen::Vector3d delta_v_;
  Eigen::Vector3d delta_rot_;
  
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_PREINTEGRATION_FACTOR_ON_MAINFOLD_H_
