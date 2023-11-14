cmake_minimum_required(VERSION 3.5.1)
project(async_stream)


# set c++ compile config
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC") # changed
set(PROTOBUF_LIBRARY protobuf::libprotobuf)

include(cmake/common.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  roslib
  morai_msgs
  cyber_perception_msgs
  perception_msgs
  geometry_msgs
  utils
  map
  tf
  autoware_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES async_stream
  # CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

find_package(Protobuf CONFIG REQUIRED)
add_subdirectory(open-simulation-interface)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/open-simulation-interface
  include
  src
)

set(OPEN_SIMULATION_INTERFACE_SRCS_PATH "${PROJECT_SOURCE_DIR}/open-simulation-interface")
set(OPEN_SIMULATION_INTERFACE_INCLUDE_PATH "${CMAKE_CURRENT_BINARY_DIR}/open-simulation-interface")

# Proto file
get_filename_component(sensorview_rpc_proto "protos/sensorview_rpc.proto" ABSOLUTE)
get_filename_component(sensorview_rpc_proto_path "${sensorview_rpc_proto}" PATH)

# Generated sources
set(sensorview_rpc_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/sensorview_rpc.pb.cc")
set(sensorview_rpc_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/sensorview_rpc.pb.h")
set(sensorview_rpc_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/sensorview_rpc.grpc.pb.cc")
set(sensorview_rpc_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/sensorview_rpc.grpc.pb.h")

add_custom_command(
      OUTPUT "${sensorview_rpc_proto_srcs}" "${sensorview_rpc_proto_hdrs}" "${sensorview_rpc_grpc_srcs}" "${sensorview_rpc_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${sensorview_rpc_proto_path}"
        -I "${OPEN_SIMULATION_INTERFACE_SRCS_PATH}"
        -I "${OPEN_SIMULATION_INTERFACE_INCLUDE_PATH}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${sensorview_rpc_proto}"
      DEPENDS "${sensorview_rpc_proto}")

## Declare a C++ library
add_library(sensorview_grpc_proto
  ${sensorview_rpc_proto_srcs}
  ${sensorview_rpc_proto_hdrs}
  ${sensorview_rpc_grpc_srcs}
  ${sensorview_rpc_grpc_hdrs}
)
target_link_libraries(sensorview_grpc_proto
  ${catkin_LIBRARIES}
  ${_REFLECTION}
  open_simulation_interface_static
)

foreach(_target
  async_streaming_server_lidar osi_bridge_main)
  add_executable(${_target}
    "src/${_target}.cc"
    src/task/task_manager.cpp
    src/utils/sensor_data_osi_converter.cpp
    src/autoware_ros_bridge.cc
    src/autoware_ros_converter.cc)
  
  target_link_libraries(${_target}
    PUBLIC
      sensorview_grpc_proto
      open_simulation_interface_static
  )
endforeach()
