/**
 * Copyright (c) 2021 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#include "range_data_matching/map/value_conversion_tables.h"

#include <random>

#include <gtest/gtest.h>

TEST(ValueConversionTableTest, EqualTables) {
  cartographer::mapping::ValueConversionTables value_conversion_table;
  const std::vector<float>* reference_table =
      value_conversion_table.GetConversionTable(0.1f, 0.1f, 0.5f);
  const std::vector<float>* test_table =
      value_conversion_table.GetConversionTable(0.1f, 0.1f, 0.5f);
  EXPECT_EQ(reference_table, test_table);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
