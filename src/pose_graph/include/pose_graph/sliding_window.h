/**
 * @file sliding_window.h
 * @author feixiang Zeng (zengfeixiang@xiaomi.com)
 * @brief
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright Copyright (c) 2022
 *
 */

// state vector: n+1 pose of lidar, ba, bg, lidar to imu
// Process of sliding window algorithm which perform as vio, laser constraint
// perform as pointcloud align with submap(just like ceres scan matching)
// imu and odom preintegration as constraint just like vio