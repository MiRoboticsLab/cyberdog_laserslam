// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#pragma once
#include "sensor/point_cloud.h"

namespace cartographer {
namespace sensor {
class FeatureExtractor {
  public:
    FeatureExtractor(int num_curve_pts, double threshold)
        : num_curve_pts_(num_curve_pts), threshold_(threshold) {}
    virtual ~FeatureExtractor() {}

    PointCloud IsCurvatureEnough(const PointCloud &pc);

  private:
    int num_curve_pts_;
    double threshold_;
};
typedef std::shared_ptr<FeatureExtractor> FeatureExtractorPtr;
typedef std::shared_ptr<const FeatureExtractor> FeatureExtractorConstPtr;
} // namespace sensor
} // namespace cartographer
