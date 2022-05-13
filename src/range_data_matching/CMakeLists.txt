cmake_minimum_required(VERSION 3.7)
project(range_data_matching)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Release")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(protos REQUIRED)
find_package(sensor REQUIRED)
find_package(common REQUIRED)
find_package(rviz_display REQUIRED)
find_package(transform REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(absl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(absl REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${sensor_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
  ${protos_INCLUDE_DIRS}
  ${rviz_display_INCLUDE_DIRS}
  ${transform_INCLUDE_DIRS}
)

message("!!!!!!!!!!!!!!!!!!")
message("${CERES_LIBRARIES}")
message("${CERES_INCLUDE_DIRS}")
message("!!!!!!!!!!!!!!!!!!")

#link_directories(${EIGEN_LIBRARY_DIRS}
#                 glog
#                 /usr/local/lib)
add_library(${PROJECT_NAME} src/grid_2d.cc                  
                            src/probability_grid_range_data_inserter_2d.cc
                            src/probability_grid.cc
                            src/probability_values.cc
                            src/ray_to_pixel_mask.cc
                            src/submap_2d.cc                            
                            src/value_conversion_tables.cc
                            src/occupied_space_cost_function_2d.cc
                            src/ceres_scan_matcher_2d.cc
                            src/correlative_scan_matcher_2d.cc
                            src/fast_correlative_scan_matcher_2d.cc
                            src/real_time_correlative_scan_matcher_2d.cc
                            src/imu_integration_midpoint.cc
                            src/transform_pc_2_image.cc
                            src/line_extraction.cc
                            src/line_extract_by_pcl.cc
                            src/filter.cc
                            src/motion_filter.cc
                            src/complementary_filter.cc)

ament_target_dependencies(${PROJECT_NAME}
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "transform"
                            "common"
                            "rviz_display"
                            "sensor"
                            "protos"
                            "absl"       
                            )                            

add_executable(integrator_test_node src/imu_integration_midpoint_test.cc)
target_link_libraries(integrator_test_node glog ${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}  )
add_executable(filter_test_node src/filter_test_node.cc)
target_link_libraries(filter_test_node glog ${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}  )
add_executable(pcl_extractor src/extract_line.cc)
target_link_libraries(pcl_extractor glog ${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}  )
add_executable(occupancy_grid_test src/occupancy_grid_test.cc)
target_link_libraries(occupancy_grid_test glog ${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} 
absl::algorithm
absl::base
absl::debugging
absl::flat_hash_map
absl::memory
absl::meta
absl::numeric
absl::str_format
absl::strings
absl::synchronization
absl::time
absl::utility)

ament_target_dependencies(integrator_test_node
         "rclcpp"
         "sensor_msgs"
         "nav_msgs"
         "geometry_msgs"
         "tf2"
         "laser_geometry"
         )
ament_target_dependencies(filter_test_node
         "rclcpp"
         "sensor_msgs"
         "nav_msgs"
         "geometry_msgs"
         "tf2"
         "laser_geometry"
         )
ament_target_dependencies(occupancy_grid_test
         "rclcpp"
         "sensor_msgs"
         "nav_msgs"
         "geometry_msgs"
         "tf2"
         "laser_geometry"
         "common"
         "sensor"
         "rviz_display"
         "Protobuf"
         )

#add_executable(xy_index_test src/xy_index_test.cc)
#target_link_libraries(xy_index_test  gtest gtest_main ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} glog)
#add_test(NAME xy_index_test COMMAND xy_index_test)

#add_executable(value_conversion_table_test src/value_conversion_table_test.cc)
#target_link_libraries(value_conversion_table_test gtest gtest_main glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME})
#add_test(NAME value_conversion_table_test COMMAND value_conversion_table_test)

add_executable(ceres_scan_matcher_test src/ceres_scan_matching_test.cc)
target_link_libraries(ceres_scan_matcher_test gtest gtest_main glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} absl::algorithm
absl::base
absl::debugging
absl::flat_hash_map
absl::memory
absl::meta
absl::numeric
absl::strings
absl::synchronization
absl::time
absl::utility)

add_test(NAME ceres_scan_matcher_test COMMAND ceres_scan_matcher_test)
ament_target_dependencies(ceres_scan_matcher_test
                           "protos"
                           "transform"
                           "common"
                           "sensor"
                           "Protobuf"
                           "absl"
                           )


#add_executable(transform_test src/transform_test.cc)
#target_link_libraries(transform_test gtest gtest_main gmock glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} absl::strings)
#add_test(NAME transform_test COMMAND transform_test)

#add_executable(transform_pc_test src/transform_pc_2_image_test.cc)
#target_link_libraries(transform_pc_test gtest gtest_main gmock glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} absl::strings ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
#add_test(NAME transform_pc_test COMMAND transform_pc_test)

#add_executable(line_extraction_test src/line_extraction_test.cc)
#target_link_libraries(line_extraction_test gtest gtest_main gmock glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} absl::strings ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
#add_test(NAME line_extraction_test COMMAND line_extraction_test)

#add_executable(filter_test src/filter_test.cc)
#target_link_libraries(filter_test gtest gtest_main gmock glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} absl::strings ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
#add_test(NAME filter_test COMMAND filter_test)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 common transform sensor Ceres PCL)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_include_directories(include)
#install(TARGETS
#        range_data_matching
#        DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY  include/${PROJECT_NAME} DESTINATION include)
#ament_export_include_directories(include)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
