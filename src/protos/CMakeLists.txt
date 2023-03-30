cmake_minimum_required(VERSION 3.7)
project(protos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf REQUIRED)

#ind_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
#set(_REFLECTION gRPC::grpc++_reflection)

#find_package(gRPC CONFIG REQUIRED)

#message(gRPC "Using gRPC ${gRPC}")

#set(_GRPC_GRPCPP gRPC::grpc++)

find_package(common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
message(Protobuf " Using Protobuf ${Protobuf_LIBRARIES}")
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${common_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Protobuf_INCLUDE_DIRS}
)

link_directories(${EIGEN_LIBRARY_DIRS}
                 glog
                 /usr/local/lib)

               
LIST(APPEND PROTO_FLAGS -I${CMAKE_SOURCE_DIR})
file(GLOB_RECURSE MAPPING_PROTOS ${CMAKE_SOURCE_DIR}/mapping/*.proto)
list(APPEND MSG_PROTOS ${MAPPING_PROTOS})
file(GLOB_RECURSE SENSOR_PROTOS ${CMAKE_SOURCE_DIR}/sensor/*.proto)
list(APPEND MSG_PROTOS ${SENSOR_PROTOS})
file(GLOB_RECURSE TRANSFORM_PROTOS ${CMAKE_SOURCE_DIR}/transform/*.proto)
list(APPEND MSG_PROTOS ${TRANSFORM_PROTOS})
set(MESSAGE_SRC "")
set(MESSAGE_HDRS "")

foreach(msg ${MSG_PROTOS})
        file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${msg})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${msg} NAME_WE)

        list(APPEND MESSAGE_SRC "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
        list(APPEND MESSAGE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

        add_custom_command(
          OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
                 "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
          COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
          ARGS --cpp_out  ${PROJECT_BINARY_DIR}
            -I ${CMAKE_SOURCE_DIR}
            ${msg}
          DEPENDS ${msg}
          COMMENT "Running C++ protocol buffer compiler on ${msg}"
          VERBATIM
        )
endforeach()



set_source_files_properties(${MESSAGE_SRC} ${MESSAGE_HDRS} PROPERTIES GENERATED TRUE)
#list(APPEND ALL_LIBRARY_HDRS ${MESSAGE_GRPC_HDRS})
#list(APPEND ALL_LIBRARY_SRCS ${MESSAGE_GRPC_SRC})
list(APPEND ALL_LIBRARY_HDRS ${MESSAGE_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${MESSAGE_SRC})
file(GLOB_RECURSE HDRS ${CMAKE_SOURCE_DIR}/*.h)
file(GLOB_RECURSE SRCS ${CMAKE_SOURCE_DIR}/src/*.cc)
list(APPEND ALL_LIBRARY_HDRS ${HDRS})
list(APPEND ALL_LIBRARY_SRCS ${SRCS})


set(INSTALL_GENERATED_HDRS ${MESSAGE_HDRS})

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

add_library(${PROJECT_NAME} ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
ament_target_dependencies(${PROJECT_NAME}
                           "Protobuf")

#add_executable(grpc_test test/client_test.cc)
#target_link_libraries(grpc_test  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  ${PROJECT_NAME}
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
#absl::utility
#glog
#${_REFLECTION}
#${_GRPC_GRPCPP}
#${_PROTOBUF_LIBPROTOBUF}
#)
#message(gRPC "Using gRPC ${_GRPC_GRPCPP}")

#add_executable(server_test test/server_test.cc)
#target_link_libraries(server_test  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  ${PROJECT_NAME}
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
#absl::utility
#glog
#${_REFLECTION}
#${_GRPC_GRPCPP}
#${_PROTOBUF_LIBPROTOBUF}
#)

#ament_target_dependencies(grpc_test
#                          "rclcpp"
#                          "Protobuf"
#                           )
                    

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 absl Protobuf)
foreach(HR ${HDRS})
  install(
    FILES ${HR}
    DESTINATION include/${PROJECT_NAME}
  )
endforeach()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin)


ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
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
