/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#ifndef POSE_GRAPH_COST_FUNCTION_SPA_COST_FUNCTION_H_
#define POSE_GRAPH_COST_FUNCTION_SPA_COST_FUNCTION_H_
#include <ceres/ceres.h>

#include "pose_graph/optimization_problem/pose_graph_components.h"

namespace cartographer {
namespace pose_graph {
namespace optimization {
ceres::CostFunction* CreateAutoDiffSpaCostFunction(const Pose& pose);

}
}  // namespace pose_graph
}  // namespace cartographer

#endif  // POSE_GRAPH_COST_FUNCTION_SPA_COST_FUNCTION_H_
