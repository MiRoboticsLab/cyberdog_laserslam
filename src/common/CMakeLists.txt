cmake_minimum_required(VERSION 3.7)
project(common)

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
find_package(rclcpp)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

link_directories(${EIGEN_LIBRARY_DIRS}
                 glog
                 /usr/local/lib)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/time.cc
                            src/task.cc
                            src/thread_pool.cc
                            src/fixed_ratio_sampler.cc)

ament_target_dependencies(${PROJECT_NAME}
             "rclcpp"
                            )                            

#add_executable(task_test src/task_test.cc)
#target_link_libraries(task_test glog gtest gtest_main gmock ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} 
#absl::algorithm
#absl::base
#absl::debugging
#absl::flat_hash_map
#absl::memory
#absl::meta
#absl::numeric
#absl::str_format
#absl::strings
#absl::synchronization
#absl::time
#absl::utility)

#add_test(NAME task_test COMMAND task_test)   
                            
#ament_target_dependencies(task_test
#                          )                            

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 absl Boost)
install(TARGETS common
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
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
