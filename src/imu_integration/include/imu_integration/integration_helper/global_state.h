/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef IMU_INTEGRATION_INTEGRATION_HELPER_GLOBAL_STATE_H_
#define IMU_INTEGRATION_INTEGRATION_HELPER_GLOBAL_STATE_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::VectorXd VXD;
typedef Eigen::MatrixXd MXD;
typedef Eigen::Quaterniond Q4D;

constexpr double kGravity = 9.81;  // gravity constant
constexpr unsigned int kStateDim = 15;
constexpr unsigned int kNoiseDim = 12;  // Dimension of state
constexpr unsigned int kPosition = 0;
constexpr unsigned int kVelocity = 3;
constexpr unsigned int kRotation = 6;
constexpr unsigned int kAccBias = 9;
constexpr unsigned int kGyroBias = 12;
constexpr unsigned int kGra = 15;  // The state from
/**
 * Define Imu Global State which should suit for any algorithm modeled by imu
 * state [P, V, Q, ba, bg, g] P is position, V is velocity, Q is rotation from
 * last frame to frame now on, ba is bias of acc, bg is bias of gyro, g is
 * gravity
 */
namespace imu_integration {
class GlobalState {
 public:
  GlobalState() { SetIdentity(); }
  GlobalState(const V3D& position, const V3D& velocity, const Q4D& rotation,
              const V3D& ba, const V3D& bg) {
    SetIdentity();
    position_ = position;
    velocity_ = velocity;
    rotation_ = rotation;
    ba_ = ba;
    bg_ = bg;
  }
  virtual ~GlobalState() {}

  void SetIdentity() {
    position_.setZero();
    velocity_.setZero();
    rotation_.setIdentity();
    ba_.setZero();
    bg_.setZero();
    gravity_ << 0.0, 0.0, -kGravity;
  }

  GlobalState& operator=(const GlobalState& other) {
    if (this == &other) return *this;

    this->position_ = other.position_;
    this->velocity_ = other.velocity_;
    this->rotation_ = other.rotation_;
    this->ba_ = other.ba_;
    this->bg_ = other.bg_;
    this->gravity_ = other.gravity_;
  }
  // state, could be get by outside, so is public
  V3D position_;
  V3D velocity_;
  Q4D rotation_;
  V3D ba_;
  V3D bg_;
  V3D gravity_;
};  // class GlobalState
}  // namespace imu_integration
#endif  // IMU_INTEGRATION_INTEGRATION_HELPER_GLOBAL_STATE_H_
