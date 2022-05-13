/*
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef RANGE_DATA_MATCHING_MAP_GRID_H_
#define RANGE_DATA_MATCHING_MAP_GRID_H_

#include <memory>
#include <utility>

namespace cartographer {
namespace mapping {

class GridInterface {
 public:
  virtual ~GridInterface() {}
};

}  // namespace mapping
}  // namespace cartographer

#endif  // RANGE_DATA_MATCHING_MAP_GRID_H_
