#include "sensor/voxel_filter.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace cartographer {
namespace mapping {
class VoxelFilterTest : public rclcpp::Node {
 public:
  VoxelFilterTest() : Node("voxel_filter") {
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&VoxelFilterTest::LaserCallBack, this,
                      std::placeholders::_1));
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
  }
  virtual ~VoxelFilterTest() {}

 private:
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    int nums = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    // points.resize(nums);
    for (int i = 0; i < nums; ++i) {
      sensor::RangefinderPoint pt;
      pt.position.x() =
          sin(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];
      pt.position.y() =
          cos(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];
      pt.position.z() = 0.0;
      points.push_back(pt);
    }
    const sensor::PointCloud cloud(points, intensities);
    sensor::PointCloud filtered_cloud = sensor::VoxelFilter(cloud, 0.05);
    sensor_msgs::msg::PointCloud2 new_cloud;
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
};

}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<cartographer::mapping::VoxelFilterTest> filter =
      std::make_shared<cartographer::mapping::VoxelFilterTest>();
  rclcpp::spin(filter);
  rclcpp::shutdown();
  return 0;
}