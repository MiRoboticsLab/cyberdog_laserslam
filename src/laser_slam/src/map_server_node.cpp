// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights
// reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "laser_slam/map_server_node.hpp"
using namespace std::chrono_literals;
namespace cartographer {
namespace laser_slam {
void MapServerNode::StartThread() {
    quit_thread_ = false;
    map_task_thread_.reset(
        new std::thread(std::bind(&MapServerNode::MapTaskThread, this)));
    map_generate_thread_.reset(
        new std::thread(std::bind(&MapServerNode::MapGenerateThread, this)));
}

void MapServerNode::StopThread() {
    quit_thread_ = true;
    while (!map_task_end_) {
        LOG(INFO) << "Map task not end";
        usleep(1000);
    }
    while (!insertion_task_end_) {
        LOG(INFO) << "insertion task not end";
        usleep(1000);
    }
    if (map_task_thread_ != nullptr) {
        if (map_task_thread_->joinable()) {
            map_task_thread_->join();
        }
    }
    if (map_generate_thread_ != nullptr) {
        if (map_generate_thread_->joinable()) {
            map_generate_thread_->join();
        }
    }
}

void MapServerNode::AddRangeDataAndPose(const sensor::RangeData &pc,
                                        const transform::Rigid3d &pose) {
    std::lock_guard<std::mutex> lk(task_mtx_);
    pose_range_datas_.push_back(PoseRangeData{pc, pose});
}

void MapServerNode::MapGenerateThread() {
    LOG(INFO) << "map generate thread";
    while (!quit_thread_) {
        map_task_end_ = false;
        if (insertion_map_ != nullptr) {
            if (num_accumulated_range_data_ > 20) {
                // first_update_ = false;
                std::unique_ptr<mapping::Submap2D> insertion_map_bk;
                {
                    std::lock_guard<std::mutex> lk(insertion_map_mtx_);
                    insertion_map_bk = std::move(insertion_map_);
                    LOG(INFO) << "insertion map unique moved";
                }
                {
                    std::lock_guard<std::mutex> lsk(range_data_accumulate_mtx_);
                    num_accumulated_range_data_ = 0;
                }
                Eigen::Array2i offset;
                mapping::CellLimits cell_limits;
                const auto &map_limits = insertion_map_bk->grid()->limits();
                insertion_map_bk->grid()->ComputeCroppedLimits(&offset,
                                                               &cell_limits);
                if (cell_limits.num_x_cells == 0 ||
                    cell_limits.num_y_cells == 0) {
                    LOG(WARNING) << "Empty grid";
                    continue;
                }
                int width = cell_limits.num_x_cells;
                int height = cell_limits.num_y_cells;
                std::vector<uint32> data(width * height);
                for (const Eigen::Array2i &xy_index :
                     mapping::XYIndexRangeIterator(cell_limits)) {
                    const Eigen::Array2i index = xy_index + offset;
                    uint32 value;
                    if (insertion_map_bk->grid()->IsKnown(index)) {
                        float probability =
                            insertion_map_bk->grid()->GetProbability(index);
                        const float pb = 1.f - probability;
                        value = common::RoundToInt(
                            255 * ((pb - mapping::kMinProbability) /
                                   (mapping::kMaxProbability -
                                    mapping::kMinProbability)));
                        if (value > 128) {
                            data[xy_index.y() * width + xy_index.x()] = 0;
                        } else {
                            data[xy_index.y() * width + xy_index.x()] = 100;
                        }
                    } else {
                        data[xy_index.y() * width + xy_index.x()] = -1;
                    }
                }
                // rotate 90 degree clockwise
                const auto old_data = data;
                data.clear();
                for (int x = 0; x < width; ++x) {
                    for (int y = height - 1; y >= 0; --y) {
                        data.push_back(old_data.at(y * width + x));
                    }
                }
                LOG(INFO) << "width is: " << width << "heght is: " << height;
                std::swap(width, height);
                LOG(INFO) << "width is: " << width << "heght is: " << height;
                LOG(INFO) << "max is: " << map_limits.max().transpose();
                const Eigen::Vector2d origin(
                    map_limits.max().x() - (offset.y() + width) * 0.05,
                    map_limits.max().y() - (offset.x() + height) * 0.05);
                double max_x = origin.x() + width * 0.05;
                double max_y = origin.y() + height * 0.05;
                double max_x_new = max_x > max_.x() ? max_x : max_.x();
                double max_y_new = max_y > max_.y() ? max_y : max_.y();
                max_ = Eigen::Vector2d(max_x_new, max_y_new);
                Eigen::Vector2d origin_new;
                origin_new.x() = origin.x() > min_.x() ? min_.x() : origin.x();
                origin_new.y() = origin.y() > min_.y() ? min_.y() : origin.y();
                min_ = origin_new;
                nav_msgs::msg::OccupancyGrid pub_grid;
                int width_final = (max_.x() - min_.x()) / 0.05;
                int height_final = (max_.y() - min_.y()) / 0.05;
                width_final = width_final + 2 * kExpandMap;
                height_final = height_final + 2 * kExpandMap;
                pub_grid.info.height = height_final;
                pub_grid.info.width = width_final;
                pub_grid.info.resolution = 0.05;
                pub_grid.info.origin.position.x = min_.x() - kExpandMap * 0.05;
                pub_grid.info.origin.position.y = min_.y() - kExpandMap * 0.05;
                pub_grid.info.origin.position.z = 0;
                pub_grid.info.origin.orientation.w = 1.0;
                pub_grid.info.origin.orientation.x = 0.0;
                pub_grid.info.origin.orientation.y = 0.0;
                pub_grid.info.origin.orientation.z = 0.0;
                pub_grid.data.clear();
                pub_grid.data.resize(width_final * height_final);
                for (int k = 0; k < pub_grid.data.size(); ++k) {
                    pub_grid.data[k] = -1;
                }
                int width_offset = std::ceil(
                    (origin.x() - pub_grid.info.origin.position.x) / 0.05);
                int height_offset = std::ceil(
                    (origin.y() - pub_grid.info.origin.position.y) / 0.05);
                LOG(INFO) << width_offset << " , " << height_offset;
                std::vector<uint32> v_data;
                v_data.reserve(width * height);
                for (int y = height - 1; y >= 0; --y) {
                    for (int x = 0; x < width; ++x) {
                        const uint32 value = data[y * width + x];
                        v_data.push_back(value);
                    }
                }
                for (int z = 0; z < height; ++z) {
                    for (int h = 0; h < width; ++h) {
                        pub_grid.data[(z + height_offset) * width_final + h +
                                      width_offset] = v_data[z * width + h];
                    }
                }
                if (publish_grid_ != nullptr) {
                    nav_msgs::msg::OccupancyGrid grid_last;
                    {
                        std::lock_guard<std::mutex> lk(ros_map_mtx_);
                        grid_last = *publish_grid_;
                    }
                    int width_offset_v =
                        std::ceil((grid_last.info.origin.position.x -
                                   pub_grid.info.origin.position.x) /
                                  0.05);
                    int height_offset_v =
                        std::ceil((grid_last.info.origin.position.y -
                                   pub_grid.info.origin.position.y) /
                                  0.05);
                    for (int y = 0; y < grid_last.info.height; ++y) {
                        for (int x = 0; x < grid_last.info.width; ++x) {
                            if (pub_grid.data[(y + height_offset_v) *
                                                  pub_grid.info.width +
                                              x + width_offset_v] == -1) {
                                pub_grid.data[(y + height_offset_v) *
                                                  pub_grid.info.width +
                                              x + width_offset_v] =
                                    grid_last
                                        .data[y * grid_last.info.width + x];
                            }
                        }
                    }
                }
                LOG(INFO) << "Draw Map End";
                std::lock_guard<std::mutex> lk(ros_map_mtx_);
                publish_grid_ =
                    std::make_shared<nav_msgs::msg::OccupancyGrid>(pub_grid);
                publish_grid_->header.frame_id = "laser_odom";
            } else {
                usleep(1000);
            }
        } else {
            usleep(1000);
        }
        map_task_end_ = true;
    }
    LOG(INFO) << "map task end";
}

void MapServerNode::MapTaskThread() {
    while (!quit_thread_) {
        insertion_task_end_ = false;
        if (quit_thread_) {
            insertion_task_end_ = true;
            return;
        }

        while (pose_range_datas_.empty()) {
            if (quit_thread_) {
                insertion_task_end_ = true;
                return;
            }

            usleep(1000);
        }
        PoseRangeData p_r;
        {
            std::lock_guard<std::mutex> lk(task_mtx_);
            p_r = pose_range_datas_.front();
        }
        auto pose = p_r.pose;
        Eigen::Vector2d origin;
        if (insertion_map_ == nullptr) {

            origin.x() = pose.translation().x();
            origin.y() = pose.translation().y();
            auto probability_grid = absl::make_unique<mapping::ProbabilityGrid>(
                mapping::MapLimits(
                    0.05, origin + 0.5 * 100 * 0.05 * Eigen::Vector2d::Ones(),
                    mapping::CellLimits(100, 100)),
                &convertion_tables_);
            auto grid = std::unique_ptr<mapping::Grid2D>(
                static_cast<mapping::Grid2D *>(probability_grid.release()));

            insertion_map_ = absl::make_unique<mapping::Submap2D>(
                origin.cast<float>(), std::move(grid), &convertion_tables_);
            LOG(INFO) << "insertion map created";
        }
        sensor::RangeData range_data;
        range_data = p_r.range_data;
        if (insertion_map_ != nullptr) {
            LOG(INFO) << "Insert range data";
            std::lock_guard<std::mutex> lk(insertion_map_mtx_);
            insertion_map_->InsertRangeData(range_data,
                                            range_data_inserter_.get());
            {
                std::lock_guard<std::mutex> lsk(range_data_accumulate_mtx_);
                ++num_accumulated_range_data_;
            }
            std::lock_guard<std::mutex> lllk(task_mtx_);
            pose_range_datas_.pop_front();
        }
        insertion_task_end_ = true;
    }
    LOG(INFO) << "map generate task end";
}
} // namespace laser_slam
} // namespace cartographer
