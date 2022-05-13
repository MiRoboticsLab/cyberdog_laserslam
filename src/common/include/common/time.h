/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef RANGE_DATA_MATCHING_COMMON_TIME_H_
#define RANGE_DATA_MATCHING_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "rclcpp/rclcpp.hpp"

#include "common/port.h"

namespace cartographer {
namespace common {

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
static inline Duration FromSeconds(double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
static inline double ToSeconds(Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
static inline Time FromUniversal(int64 ticks) { return Time(Duration(ticks)); }

// Outputs the Universal Time Scale timestamp for a given Time.
static inline int64 ToUniversal(Time time) {
  return time.time_since_epoch().count();
}

// For logging and unit tests, outputs the timestamp integer.
static inline std::ostream& operator<<(std::ostream& os, Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

// CPU time consumed by the thread so far, in seconds.
double GetThreadCpuTimeSeconds();

static inline rclcpp::Time ToRosTime(const Time& time) {
  int64_t uts_timestamp = ToUniversal(time);
  int64_t ns_since_unix_epoch =
      (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) *
      100ll;
  rclcpp::Time ros_time(ns_since_unix_epoch, RCL_SYSTEM_TIME);
  return ros_time;
}

static inline Time FromRosTime(const rclcpp::Time& ros_time) {
  return FromUniversal(
      (ros_time.seconds() + kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (ros_time.nanoseconds() + 50) / 100);
}
}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
