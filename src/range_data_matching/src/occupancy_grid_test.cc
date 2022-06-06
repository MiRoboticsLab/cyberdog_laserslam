/**
 * Copyright (c) 2022 XiaoMi
 *
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 *
 */
#include "range_data_matching/map/submap_2d.h"

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("map") {
    publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("topic", 10);
  }

  void Publish(const nav_msgs::msg::OccupancyGrid& map) {
    publisher_->publish(map);
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  cartographer::SubMapParam param;
  param.grid_insert_type = 0;
  param.num_range_data = 10;
  param.grid_type = 0;
  param.resolution = 0.05;
  param.probability_insert_param.hit_probability = 0.55;
  param.probability_insert_param.miss_probability = 0.45;

  cartographer::mapping::ActiveSubmaps2D active_submap_2d(param);
  cartographer::sensor::RangeData range_data;
  range_data.origin = Eigen::Vector3f::Zero();
  std::vector<cartographer::sensor::RangefinderPoint> points;
  for (int i = 0; i < 10; ++i) {
    cartographer::sensor::RangefinderPoint pt{
        Eigen::Vector3f(0.1f * i, 0.1f * i, 0.f)};
    points.push_back(pt);
  }
  cartographer::sensor::PointCloud pc(points);
  range_data.returns = pc;
  rclcpp::Time time;
  // std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid =
  //     active_submap_2d.InsertRangeData(range_data)
  //         .front()
  //         ->grid()
  //         ->ToRosOccupancyMsg(0.05, "laser", time);
  // Publisher pub;
  //   while (1) {
  // pub.Publish(*grid);
  //   }
  // std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid1 =
  //     active_submap_2d.InsertRangeData(range_data)
  //         .front()
  //         ->grid()
  //         ->ToRosOccupancyMsg(0.05, "laser", time);
  // pub.Publish(*grid1);
  rclcpp::shutdown();
  return 0;
}
