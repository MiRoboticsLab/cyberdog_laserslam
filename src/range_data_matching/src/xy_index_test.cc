/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include <set>
#include <memory>

#include <gtest/gtest.h>

#include "range_data_matching/map/xy_index.h"
#include "range_data_matching/map/map_limits.h"
#include "range_data_matching/map/submap_2d.h"

TEST(cell_limit, structor) {
  int x = 15;
  int y = 15;
  cartographer::mapping::CellLimits cell_limit(x, y);
  EXPECT_EQ(cell_limit.num_x_cells, x);
  EXPECT_EQ(cell_limit.num_y_cells, y);
  cartographer::mapping::XYIndexRangeIterator iter(cell_limit);
  ++iter;
  Eigen::Array2i xy_index = *iter;
  EXPECT_EQ(xy_index.x(), 1);
  EXPECT_EQ(xy_index.y(), 0);
}

TEST(map_limit, function) {
  double resolution = 0.05;
  Eigen::Vector2d max;
  max << 1.3, 1.3;
  cartographer::mapping::CellLimits cell_limit(15, 15);
  cartographer::mapping::MapLimits map_limit(resolution, max, cell_limit);
  Eigen::Vector2f point(0.5, 0.5);
  Eigen::Array2i index = map_limit.GetCellIndex(point);
  bool is_contain = map_limit.Contains(index);
  EXPECT_EQ(0, 0);
  EXPECT_FALSE(is_contain);
}

TEST(submap_2d, active_map) {
  double hit_probability = 0.6;
  double miss_probability = 0.4;
  bool insert_free = true;
  cartographer::ProbabilityInserterParam p_param;
  p_param.hit_probability = hit_probability;
  p_param.miss_probability = miss_probability;
  p_param.insert_free = insert_free;
  int grid_insert_type = 0;
  int num_range_data = 10;
  int grid_type = 0;
  float resolution = 0.05;
  cartographer::SubMapParam s_param;
  s_param.grid_insert_type = grid_insert_type;
  s_param.num_range_data = num_range_data;
  s_param.grid_type = grid_type;
  s_param.resolution = resolution;
  s_param.probability_insert_param = p_param;
  cartographer::mapping::ActiveSubmaps2D submaps(s_param);
  std::set<std::shared_ptr<const cartographer::mapping::Submap2D>> all_submaps;
  for (int i = 0; i != 1000; ++i) {
    auto insertion_submaps =
        submaps.InsertRangeData({Eigen::Vector3f::Zero(), {}, {}});
    for (const auto& submap : insertion_submaps) {
      all_submaps.insert(submap);
    }
    if (submaps.submaps().size() > 1) {
      EXPECT_LE(num_range_data, submaps.submaps().front()->num_range_data());
    }
  }
  EXPECT_EQ(2, submaps.submaps().size());
  int correct_num_finished_submaps = 0;
  int num_unfinished_submaps = 0;
  for (const auto& submap : all_submaps) {
    if (submap->num_range_data() == num_range_data * 2) {
      ++correct_num_finished_submaps;
    } else {
      EXPECT_EQ(num_range_data, submap->num_range_data());
      ++num_unfinished_submaps;
    }
  }
  EXPECT_EQ(correct_num_finished_submaps, all_submaps.size() - 1);
  EXPECT_EQ(1, num_unfinished_submaps);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
