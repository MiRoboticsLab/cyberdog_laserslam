cmake_minimum_required(VERSION 3.7)
project(laser_slam)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
#set(_REFLECTION gRPC::grpc++_reflection)
#find_package(gRPC CONFIG REQUIRED)
#message(gRPC "Using gRPC ${gRPC_VERSION}")
#set(_GRPC_GRPCPP gRPC::grpc++)
find_package(absl REQUIRED)
find_package(common REQUIRED)
find_package(pose_extrapolator REQUIRED)
find_package(range_data_matching REQUIRED)
find_package(rviz_display REQUIRED)
find_package(pose_graph REQUIRED)


# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${sensor_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
  ${protos_INCLUDE_DIRS}
  ${transform_INCLUDE_DIRS}
  ${pose_extrapolator_INCLUDE_DIRS}
  ${rviz_display_INCLUDE_DIRS}
  ${range_data_matching_INCLUDE_DIRS}
  ${pose_graph_INCLUDE_DIRS}
)


link_directories(${EIGEN_LIBRARY_DIRS}
                 glog
                 /usr/local/lib)

add_library(${PROJECT_NAME} src/grid_for_navigation.cc
                            src/final_map_generator.cc
                            src/local_slam.cc
                            src/back_end.cc
                            src/map_builder.cc
                            src/map_loader.cc
                            src/localization.cc
                            src/pose_recorder.cc)

ament_target_dependencies(${PROJECT_NAME}
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "visualization"
                            "tf2"
                            "laser_geometry"
                            "common"
                            "protos"
                            "sensor"
                            "transform"
                            "Protobuf"
                            )                            

add_executable(local_slam_test src/local_slam_test.cc)
target_link_libraries(local_slam_test gtest gtest_main glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} 
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF})

add_test(NAME local_slam_test COMMAND local_slam_test)   

ament_target_dependencies(local_slam_test
                           "Protobuf"
                           "transform"
                           "common"
                           "sensor"
                           "pose_extrapolator"
                           "range_data_matching"
                           "pose_graph"
                           "rviz_display"
                           )

add_executable(local_slam_test_node src/local_slam_test_node.cc)
target_link_libraries(local_slam_test_node glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES} 
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF})           

ament_target_dependencies(local_slam_test_node
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "visualization"
                            "tf2"
                            "laser_geometry"
                            "Protobuf"
                            "pose_graph"
                            "common"
                            "protos"
                            "sensor"
                            "transform"
                            "pose_extrapolator"
                            "range_data_matching"
                            )

add_executable(demo_front_end demo_front_end/front_end_demo.cc)
target_link_libraries(demo_front_end glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES}
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF}
${Boost_LIBRARIES})  

ament_target_dependencies(demo_front_end
                            "rclcpp"
                            "sensor_msgs"
                            "nav_msgs"
                            "geometry_msgs"
                            "tf2"
                            "laser_geometry"
                            "visualization"
                            "Protobuf"
                            "Boost"
                            "pose_graph"
                            "common"                           
                            "sensor"
                            "transform"
                            "pose_extrapolator"
                            "range_data_matching"  
                            "protos"        
                           ) 

add_executable(demo demo_front_end/localization_demo.cc)
target_link_libraries(demo glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES}
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF}
${Boost_LIBRARIES})  
                           
message("!!!!!!!!!!!!!!!!!!")
message("${Boost_LIBRARIES}")
message("${BOOST_INCLUDE_DIRS}")
message("!!!!!!!!!!!!!!!!!!")

ament_target_dependencies(demo
                          "rclcpp"
                          "sensor_msgs"
                          "nav_msgs"
                          "geometry_msgs"
                          "tf2"
                          "laser_geometry"
                          "visualization"
                          "Protobuf"
                          "Boost"
                          "pose_graph"
                          "common"                           
                          "sensor"
                          "transform"
                          "pose_extrapolator"
                          "range_data_matching"  
                          "protos"        
                          ) 

add_executable(deserializer src/test_pose_graph_reader.cc)
target_link_libraries(deserializer glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES} 
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF}
${Boost_LIBRARIES})  

ament_target_dependencies(deserializer
                          "rclcpp"
                          "sensor_msgs"
                          "nav_msgs"
                          "geometry_msgs"
                          "tf2"
                          "laser_geometry"
                          "visualization"
                          "Protobuf"
                          "Boost"
                          "pose_graph"
                          "common"                           
                          "sensor"
                          "transform"
                          "pose_extrapolator"
                          "range_data_matching"  
                          "protos" 
                          ) 
                           

add_executable(pose_graph demo_front_end/demo_with_pose_graph.cc)
target_link_libraries(pose_graph glog ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES} 
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
absl::utility
#${_REFLECTION}
#${_GRPC_GRPCPP}
${_PROTOBUF_LIBPROTOBUF})   
                                                     
ament_target_dependencies(pose_graph
                           "rclcpp"
                           "sensor_msgs"
                           "nav_msgs"
                           "geometry_msgs"
                           "tf2"
                           "laser_geometry"
                           "visualization"
                           "Protobuf"
                           "pose_graph"
                           "common"                           
                           "sensor"
                           "transform"
                           "pose_extrapolator"
                           "range_data_matching"  
                           "protos"
                                                     )
#install(TARGETS
#        laser_slam
#        DESTINATION lib/${PROJECT_NAME})
                            
#install(DIRECTORY  include/${PROJECT_NAME} DESTINATION include)
#ament_export_include_directories(include) 

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 absl)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(TARGETS
        demo_front_end
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        pose_graph
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        demo
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME})

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
