/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
