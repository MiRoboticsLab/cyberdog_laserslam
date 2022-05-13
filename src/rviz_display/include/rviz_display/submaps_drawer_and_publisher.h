/**
 * Copyright (c) 2022 XiaoMi
 * 
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 * 
 */
#ifndef RVIZ_DISPLAY_SUBMAPS_DRAWER_H_
#define RVIZ_DISPLAY_SUBMAPS_DRAWER_H_

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace cartographer {
namespace rviz_display {
class SubmapDrawerAndPublisher {
public:
SubmapDrawerAndPublisher() {}
virtual ~SubmapDrawerAndPublisher() {}


};

    

}
}


#endif // RVIZ_DISPLAY_SUBMAPS_DRAWER_H_
