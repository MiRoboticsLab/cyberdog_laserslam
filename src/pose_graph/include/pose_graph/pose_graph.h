/**
 * Copyright (c) 2022 XiaoMi
 * 
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 * 
 */
#ifndef POSE_GRAPH_POSE_GRAPH_H_
#define POSE_GRAPH_POSE_GRAPH_H_
#include "pose_graph/data_set/pose_graph_data.h"



namespace cartographer {
namespace pose_graph {
namespace optimization{
 class PoseGraphOpt {
  public:
   PoseGraphOpt();
   virtual ~PoseGraphOpt();

   NodeId AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data);



   private:
   PoseGraphData data_;
 };
}
}
}



#endif // POSE_GRAPH_POSE_GRAPH_H_
