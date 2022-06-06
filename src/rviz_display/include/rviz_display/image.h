/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
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

// // Takes ownership.
// UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t* surface);

// // std::unique_ptr for Cairo contexts.
// using UniqueCairoPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// // Takes ownership.
// UniqueCairoPtr MakeUniqueCairoPtr(cairo_t* surface);

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

  // void WritePng(FileWriter* const file_writer);

  // Returns a pointer to a cairo surface that contains the current pixel data.
  // The 'Image' object must therefore outlive the returned surface object. It
  // is undefined behavior to call any of the mutating functions while a pointer
  // to this surface is alive.
  // UniqueCairoSurfacePtr GetCairoSurface();

 private:
  int width_;
  int height_;
  std::vector<uint32> pixels_;
};
}  // namespace rviz_display
}  // namespace cartographer

#endif  // RVIZ_DISPLAY_IMAGE_H_
