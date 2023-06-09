cmake_minimum_required(VERSION 3.7)
project(pose_extrapolator)

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
find_package(pcl_msgs REQUIRED)
#find_package(transform REQUIRED)
#find_package(common REQUIRED)
find_package(sensor REQUIRED)

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
  ${sensor_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
  ${protos_INCLUDE_DIRS}
  ${transform_INCLUDE_DIRS}
)

link_directories(${EIGEN_LIBRARY_DIRS}
                 glog
                 /usr/local/lib)

                 
add_library(${PROJECT_NAME} src/complementary_filter.cc
                            src/imu_integration_midpoint.cc
                            src/pose_extrapolator_interface.cc
                            src/pose_extrapolator.cc
                            src/tracker.cc
                            src/imu_tracker.cc
                            src/measurement_collector.cc)
                    
ament_target_dependencies(${PROJECT_NAME}
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "tf2"
                            "laser_geometry"
                            "common"
                            "sensor"
                            "transform"
                            "absl"
			    "pcl_msgs"
                           )                          

add_executable(tracker_test_node src/tracker_test.cc)
target_link_libraries(tracker_test_node  ${PROJECT_NAME} ${common_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} glog absl::strings)   

ament_target_dependencies(tracker_test_node
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "tf2"
                            "laser_geometry"
                            "common"
                            "sensor"
                            "transform"
                            )


ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 absl common transform sensor range_data_matching)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
                            
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
                            
ament_export_include_directories(include)                          

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
