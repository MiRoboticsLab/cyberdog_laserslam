cmake_minimum_required(VERSION 3.7)
project(transform)

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
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(geometry_msgs)
find_package(Eigen3 REQUIRED)
find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
find_package(protos REQUIRED)
find_package(common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${protos_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)
file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/transform/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cc")

link_directories(${EIGEN_LIBRARY_DIRS}
                 glog
                 /usr/local/lib)

add_library(${PROJECT_NAME} ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
ament_target_dependencies(${PROJECT_NAME}
                          "geometry_msgs"
                          "common"
                          "protos"
                          "Protobuf"
                           )

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 absl common protos)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin)

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
