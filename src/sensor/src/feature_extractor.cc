// Copyright (C) [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "sensor/feature_extractor.h"

namespace cartographer {
namespace sensor {
PointCloud FeatureExtractor::IsCurvatureEnough(const PointCloud &pc) {
    size_t pc_size = pc.size();
    PointCloud pc_r;
    std::vector<sensor::RangefinderPoint> pts;
    if (pc_size < 6)
        return pc_r;
    int num_curve_pts = 0;
    for (size_t i = 5; i < pc_size - 5; ++i) {
        double diff_x = pc[i - 5].position.x() + pc[i - 4].position.x() +
                        pc[i - 3].position.x() + pc[i - 2].position.x() +
                        pc[i - 1].position.x() - 10 * pc[i].position.x() +
                        pc[i + 1].position.x() + pc[i + 2].position.x() +
                        pc[i + 3].position.x() + pc[i + 4].position.x() +
                        pc[i + 5].position.x();
        double diff_y = pc[i - 5].position.y() + pc[i - 4].position.y() +
                        pc[i - 3].position.y() + pc[i - 2].position.y() +
                        pc[i - 1].position.y() - 10 * pc[i].position.y() +
                        pc[i + 1].position.y() + pc[i + 2].position.y() +
                        pc[i + 3].position.y() + pc[i + 4].position.y() +
                        pc[i + 5].position.y();
        double curvature = diff_x * diff_x + diff_y * diff_y;
        if (curvature > threshold_) {
            ++num_curve_pts;
            pts.push_back(pc[i]);
        }
    }
    LOG(INFO) << "curvature pts num is: " << num_curve_pts;
    // if (num_curve_pts < num_curve_pts_) {
    //     return pc_r;
    // }
    // return true;
    PointCloud pp(pts);
    return pp;
}
} // namespace sensor
} // namespace cartographer
