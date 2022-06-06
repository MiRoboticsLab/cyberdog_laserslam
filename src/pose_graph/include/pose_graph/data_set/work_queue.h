/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#ifndef POSE_GRAPH_DATA_SET_WORK_QUEUE_H
#define POSE_GRAPH_DATA_SET_WORK_QUEUE_H

#include <chrono>
#include <deque>
#include <functional>

namespace cartographer {
namespace pose_graph {
namespace optimization {

struct WorkItem {
  enum class Result {
    kDoNotRunOptimization,
    kRunOptimization,
  };

  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};

using WorkQueue = std::deque<WorkItem>;
}  // namespace optimization
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_DATA_SET_WORK_QUEUE_H
