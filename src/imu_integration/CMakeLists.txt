cmake_minimum_required(VERSION 3.7)
project(imu_integration)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${EIGEN_LIBRARY_DIRS}
                 glog)

add_executable(${PROJECT_NAME}_node
      src/imu_odom_eskf_node.cc
      src/imu_odometry_eskf.cc
      src/error_state_kalman_filter.cc
      src/imu_integration_midpoint_eskf.cc)

ament_target_dependencies(${PROJECT_NAME}_node
         "rclcpp"
         "sensor_msgs"
         "nav_msgs"
         "geometry_msgs"
         )

target_link_libraries(${PROJECT_NAME}_node
         ${EIGEN_LIBRARIES}
         glog)

install(TARGETS
         imu_integration_node
         DESTINATION lib/${PROJECT_NAME})
    
install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params
         DESTINATION share/${PROJECT_NAME})
          
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
