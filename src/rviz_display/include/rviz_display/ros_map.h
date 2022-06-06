/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RVIZ_DISPLAY_ROS_MAP_H
#define RVIZ_DISPLAY_ROS_MAP_H

#include <string>

#include <eigen3/Eigen/Core>

#include "rviz_display/file_writer.h"
#include "rviz_display/image.h"

namespace cartographer {
namespace rviz_display {

// Write 'image' as a pgm into 'file_writer'. The resolution is used in the
// comment only'
void WritePgm(const Image& image, const double resolution,
              FileWriter* file_writer);

// Write the corresponding yaml into 'file_writer'.
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename, FileWriter* file_writer);

}  // namespace rviz_display
}  // namespace cartographer

#endif  // RVIZ_DISPLAY_ROS_MAP_H
