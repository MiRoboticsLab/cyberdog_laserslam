/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "sensor/voxel_filter.h"

#include <cmath>
#include <random>
#include <utility>
#include <absl/container/flat_hash_map.h>
#include <glog/logging.h>

namespace cartographer {
namespace sensor {
namespace {
PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  return point_cloud.copy_if([max_range](const RangefinderPoint& point) {
    return point.position.norm() <= max_range;
  });
}

PointCloud AdaptivelyVoxelFiltered(const PointCloud& point_cloud,
                                   const AdaptiveVoxelFilterParam& param) {
  if (point_cloud.size() <= param.min_num_points) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud result = VoxelFilter(point_cloud, param.max_length);
  if (result.size() >= param.min_num_points) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = param.max_length;
       high_length > 1e-2f * param.max_length; high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (result.size() >= param.min_num_points) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        if (candidate.size() >= param.min_num_points) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}
using VoxelKeyType = uint64_t;

VoxelKeyType GetVoxelCellIndex(const Eigen::Vector3f& point,
                               const float resolution) {
  const Eigen::Array3f index = point.array() / resolution;
  const uint64_t x = common::RoundToInt(index.x());
  const uint64_t y = common::RoundToInt(index.y());
  const uint64_t z = common::RoundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<T>& point_cloud, const float resolution,
    PointFunction&& point_function) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  absl::flat_hash_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = i;
    } else {
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

template <class T, class PointFunction>
std::vector<T> RandomizedVoxelFilter(const std::vector<T>& point_cloud,
                                     const float resolution,
                                     PointFunction&& point_function) {
  const std::vector<bool> points_used =
      RandomizedVoxelFilterIndices(point_cloud, resolution, point_function);

  std::vector<T> results;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      results.push_back(point_cloud[i]);
    }
  }
  return results;
}
}  // namespace

PointCloud VoxelFilter(const PointCloud& points, const double resolution) {
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      points.points(), resolution,
      [](const RangefinderPoint& point) { return point.position; });
  std::vector<RangefinderPoint> filtered_points;
  for (size_t i = 0; i < points.size(); ++i) {
    if (points_used[i]) {
      filtered_points.push_back(points[i]);
    }
  }
  std::vector<float> filtered_intensities;
  if (not points.intensities().empty()) {
    for (size_t i = 0; i < points.intensities().size(); i++) {
      if (points_used[i]) {
        filtered_intensities.push_back(points.intensities()[i]);
      }
    }
  }
  return PointCloud(std::move(filtered_points),
                    std::move(filtered_intensities));
}

PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
                               const AdaptiveVoxelFilterParam& param) {
  return AdaptivelyVoxelFiltered(FilterByMaxRange(point_cloud, param.max_range),
                                 param);
}
}  // namespace sensor
}  // namespace cartographer
