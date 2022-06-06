/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#include "range_data_matching/map/value_conversion_tables.h"

#include <absl/memory/memory.h>
#include <glog/logging.h>

namespace cartographer {
namespace mapping {
namespace {

constexpr uint16 kUpdateMarker = 1u << 15;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  size_t num_values = std::numeric_limits<uint16>::max() + 1;
  result->reserve(num_values);
  for (size_t value = 0; value != num_values; ++value) {
    result->push_back(SlowValueToBoundedFloat(
        static_cast<uint16>(value) & ~kUpdateMarker, unknown_value,
        unknown_result, lower_bound, upper_bound));
  }
  return result;
}
}  // namespace

const std::vector<float>* ValueConversionTables::GetConversionTable(
    float unknown_result, float lower_bound, float upper_bound) {
  std::tuple<float, float, float> bounds =
      std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    auto insertion_result = bounds_to_lookup_table_.emplace(
        bounds, PrecomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                              upper_bound));
    return insertion_result.first->second.get();
  } else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace mapping
}  // namespace cartographer
