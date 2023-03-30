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
#ifndef RVIZ_DISPLAY_IMAGE_H_
#define RVIZ_DISPLAY_IMAGE_H_
#include <array>
#include <cstdint>
#include <vector>
#include <cairo/cairo.h>
#include <glog/logging.h>

#include "common/port.h"
#include "rviz_display/file_writer.h"

namespace cartographer {
namespace rviz_display {
constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

using UniqueCairoSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;


using Uint8Color = std::array<uint8, 3>;

class Image {
 public:
  // explicit Image(UniqueCairoSurfacePtr surface);
  Image(int width, int height);

  const uint32 GetPixel(int x, int y) const;

  std::vector<uint32> pixel_data() const { return pixels_; }

  void SetPixel(int x, int y, const uint32& color);

  void Rotate90DegreeClockWise();

  int width() const { return width_; }

  int height() const { return height_; }

 private:
  int width_;
  int height_;
  std::vector<uint32> pixels_;
};
}  // namespace rviz_display
}  // namespace cartographer

#endif  // RVIZ_DISPLAY_IMAGE_H_
