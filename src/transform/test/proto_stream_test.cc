/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */

#include "protos/proto_stream.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include "common/port.h"
#include "mapping/pose_graph.pb.h"

namespace cartographer {
class ProtoStreamTest : public ::testing::Test {
 protected:
  void SetUp() override { test_directory_ = "/home/zfx/"; }

  std::string test_directory_;
};

TEST_F(ProtoStreamTest, WriteAndReadBack) {
  const std::string test_file = test_directory_ + "test_trajectory.pbstream";
  {
    stream::ProtoStreamWriter writer(test_file);
    protos::mapping::proto::PoseGraph pose_graph;
    pose_graph.set_trajectory_id(1);
    writer.WriteProto(pose_graph);

    ASSERT_TRUE(writer.Close());
  }
  {
    stream::ProtoStreamReader reader(test_file);
    protos::mapping::proto::PoseGraph pose_graph;
    ASSERT_TRUE(reader.ReadProto(&pose_graph));
    LOG(INFO) << "trajectory: " << pose_graph.DebugString();
    ASSERT_EQ(1, pose_graph.trajectory_id());

    // protos::mapping::proto::Trajectory trajectory;
    // EXPECT_FALSE(reader.ReadProto(&trajectory));
  }
  // remove(test_file.c_str());
}
}  // namespace cartographer
