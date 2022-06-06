#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <glog/logging.h>
#include <math.h>
#include "range_data_matching/line_segmentation/line_extract_by_pcl.h"
int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::ifstream fin("/home/zfx/pc.txt", std::ios::in);
  if (!fin.is_open()) {
    LOG(INFO) << "file not open";
  }
  std::vector<Eigen::Vector2d> points;
  std::string buf;
  std::string x, y;
  double x_pt, y_pt;
  bool change = false;
  while (std::getline(fin, buf)) {
    for (int i = 0; i < buf.size(); ++i) {
      if (buf[i] == ' ') {
        change = true;
        continue;
      }
      if (not change) {
        x = x + buf[i];
      } else {
        y = y + buf[i];
      }
    }
    change = false;
    x_pt = std::atof(x.c_str());
    y_pt = std::atof(y.c_str());
    Eigen::Vector2d pt(x_pt, y_pt);
    points.push_back(pt);
    x.clear();
    y.clear();
  }
  int min_points = 10;

  cartographer::line_extration::LineExtractByPCL extractor(true, 0.03, 0, 0, 10,
                                                           0.1, 25);
  std::vector<cartographer::line_extration::Line<double>> lines =
      extractor.Extract(points);
  LOG(INFO) << "extracted line num is: " << lines.size();
  //   std::ofstream output;
  //   output.open("/home/zfx/lines.txt", std::ios::out | std::ios::trunc);
  //   output << std::fixed;
  //   for (int i = 0; i < lines.size(); ++i) {
  //     output << std::setprecision(8) << lines[i].a.x() << " " <<
  //     lines[i].a.y()
  //            << " " << lines[i].b.x() << " " << lines[i].b.y() << std::endl;
  //   }
  //   output.close();
  cloud_boundary->width = points.size();
  cloud_boundary->height = 1;
  cloud_boundary->points.resize(cloud_boundary->width * cloud_boundary->height);
  for (size_t g = 0; g < points.size(); ++g) {
    cloud_boundary->points[g].x = points[g].x();
    cloud_boundary->points[g].y = points[g].y();
    cloud_boundary->points[g].z = 0.0;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line_rgb(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(
      new pcl::PointCloud<pcl::PointXYZ>);

  //点云索引相关容器
  pcl::ModelCoefficients::Ptr coefficients(
      new pcl::ModelCoefficients);  //创建一个模型参数对象，用于记录结果
  pcl::PointIndices::Ptr inliers(
      new pcl::PointIndices);  // inliers表示误差能容忍的点记录的是点云的序号
  pcl::ExtractIndices<pcl::PointXYZ> extract;  //按照点云提取器

  //直线分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;  // 创建一个分割器
  seg.setOptimizeCoefficients(
      true);  // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
  seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
  seg.setMethodType(pcl::SAC_RANSAC);    //分割方法：随机采样法
  seg.setDistanceThreshold(
      0.07);  //设置误差容忍范围，也就是阈值 这东西不能太小，自己多试试

  //依次从原始点集中分割得到多组直线点
  int i = 0,
      nr_points = cloud_boundary->size();  //分割前，边界点的原始点云的数量
  int index = 0;
  while (cloud_boundary->size() >
         0.05 * nr_points)  //如果点云剩余不足10%，就停止分割
  {
    seg.setInputCloud(cloud_boundary);  //输入点云
    seg.segment(*inliers, *coefficients);
    //分割点云，直线和直线的点法式参数
    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset."
                << std::endl;  //报错
      break;                   //中断循环
    }
    //按照索引提取点云模块
    extract.setInputCloud(cloud_boundary);  //待提取点云
    extract.setIndices(inliers);            //平面点云对应的索引
    extract.setNegative(false);
    extract.filter(*cloud_line);  //输出对应点云到cloud_line容器

    // std::ofstream output;
    // output.open("/home/zfx/pcl_line" + std::to_string(index) + ".txt",
    //             std::ios::out | std::ios::trunc);
    // output << std::fixed;
    // for (int i = 0; i < (cloud_line->width * cloud_line->height); ++i) {
    //   output << std::setprecision(8) << cloud_line->points[i].x << " "
    //          << cloud_line->points[i].y << std::endl;
    // }
    // output.close();
    index = index + 1;
    std::cerr << "直线点云数量" << cloud_line->width * cloud_line->height
              << " data points." << std::endl;
    //转换成彩色点云
    int Random_color_r, Random_color_g, Random_color_b;
    Random_color_r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
    Random_color_g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
    Random_color_b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
    for (int i = 0; i < (cloud_line->width * cloud_line->height); i++) {
      pcl::PointXYZRGB point;
      point.x = cloud_line->points[i].x;
      point.y = cloud_line->points[i].y;
      point.z = cloud_line->points[i].z;
      point.r = Random_color_r;
      point.g = Random_color_g;
      point.b = Random_color_b;
      cloud_line_rgb->push_back(point);
    }
    //结果点云文件输出
    // std::stringstream ss;  //重复输出平面，一个平面对应一个pcd文件
    // ss << "/home/zfx/" << i << ".png";
    // pcl::io::savePCDFileASCII(ss.str(), *cloud_line_rgb);
    // cloud_line_rgb->clear(); //我自己弄的这个需要每次循环的时候手动清空

    //剩余待分割点云的替换更新
    extract.setNegative(true);
    extract.filter(*cloud_filter);  //输出索引外的点云
    cloud_boundary.swap(cloud_filter);
    //用剩余点云替换掉原始点云，下一次循环是从剩余点云中分割平面
    i++;  //第几组直线点集合
  }
  std::cout << "完成" << std::endl;
  system("pause");  //防止控制台窗口一闪而过
  return 0;
}
