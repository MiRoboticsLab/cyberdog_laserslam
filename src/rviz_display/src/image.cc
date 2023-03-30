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
#include "rviz_display/image.h"

namespace cartographer {
namespace rviz_display {

Image::Image(int width, int height)
    : width_(width), height_(height), pixels_(width * height, 0) {}


void Image::SetPixel(int x, int y, const uint32& color) {
  pixels_[y * width_ + x] = color;
}

const uint32 Image::GetPixel(int x, int y) const {
  return pixels_[y * width_ + x];
}

void Image::Rotate90DegreeClockWise() {
  const auto old_pixels = pixels_;
  pixels_.clear();
  for (int x = 0; x < width_; ++x) {
    for (int y = height_ - 1; y >= 0; --y) {
      pixels_.push_back(old_pixels.at(y * width_ + x));
    }
  }
  std::swap(width_, height_);
}

}  // namespace rviz_display
}  // namespace cartographer
