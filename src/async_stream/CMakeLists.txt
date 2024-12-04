cmake_minimum_required(VERSION 3.5.1)
project(async_stream)


# set c++ compile config
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC")
set(PROTOBUF_LIBRARY protobuf::libprotobuf)

include(cmake/common.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  roslib
  morai_msgs
  geometry_msgs
  tf
  autoware_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES async_stream
  # CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

# catkin_python_setup()

# Python wrapper Setup
## before catkin_make, `sudo apt-get install pybind11-dev`.
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)

find_package(Protobuf CONFIG REQUIRED)
add_subdirectory(open-simulation-interface)
# add_subdirectory(osi-sensor-model-packaging)

# set(OSMP_PATH "${CMAKE_CURRENT_BINARY_DIR}/../../osi-sensor-model-packaging/examples/build/OSMPDummySensor")
set(OSMP_PATH "${CMAKE_CURRENT_BINARY_DIR}/osi-sensor-model-packaging/OSMPDummySensor")
include_directories(${OSMP_PATH}/buildfmu/sources)
# link_directories(${OSMP_PATH})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/open-simulation-interface
  ${PYTHON_INCLUDE_DIRS}
  include
  src
  ${PROJECT_SOURCE_DIR}/fmi-standard/headers
  ${PROJECT_SOURCE_DIR}/src/OSMPDummySensor
  lib
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

#Apollo proto code Generation
set(apllo_proto_srcs)

set(apollo_protos_list
  # common
  apollo_header
  apollo_error_code
  apollo_geometry
  apollo_vehicle_signal
  apollo_drive_state
  apollo_vehicle_config
  apollo_pnc_point
  # gnss
  apollo_gnss_best_pose
  apollo_imu
  apollo_ins
  # localization
  apollo_pose
  apollo_gps
  apollo_corimu
  # canbus
  apollo_chassis
  # map
  apollo_map_id
  apollo_map_geometry
  apollo_map_lane
  # perception
  apollo_traffic_light_detection
  apollo_perception_obstacle
  # control
  apollo_pad_msg
  apollo_input_debug
  apollo_control_cmd
  apollo_udp_bridge_remote_info
  apollo_monitor_log
  # camera
  apollo_sensor_image
  # lidar (VLP128)
  apollo_velodyne
)

foreach(protos_name ${apollo_protos_list})
  set(PROTO_SRC ${CMAKE_CURRENT_BINARY_DIR}/${protos_name}.pb.cc)
  set(PROTO_HDR ${CMAKE_CURRENT_BINARY_DIR}/${protos_name}.pb.h)
  get_filename_component(PROTO_FILE "protos/apollo_proto/${protos_name}.proto" ABSOLUTE)
  get_filename_component(PROTO_FILE_PATH "${PROTO_FILE}" PATH)

  add_custom_command(
    OUTPUT "${PROTO_SRC}" "${PROTO_HDR}"
    COMMAND protoc
    ARGS -I "${PROTO_FILE_PATH}"
      --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
      "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
  )
  list(APPEND apllo_proto_srcs ${PROTO_SRC} ${PROTO_HDR})
endforeach()

add_library(apollo_protos
  ${apllo_proto_srcs}
)

target_link_libraries(apollo_protos
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARY}
)

set(OSMP_srcs
  src/OSMPDummySensor/OSMPDummySensor.cpp)

foreach(_target
  async_streaming_server_lidar osi_bridge_main)
  add_executable(${_target}
    "src/${_target}.cc"
    src/osi_client.cc
    src/task/task_manager.cpp
    src/utils/sensor_data_osi_converter.cpp
    src/autoware_ros_bridge.cc
    src/autoware_ros_converter.cc
    src/keti_ros_bridge.cc
    src/keti_ros_converter.cc
    src/morai_ros_bridge.cc
    src/osi_server.cc
    src/utils/apollo_bridge_header.cc
    src/utils/apollo_bridge_buffer.cc
    src/utils/apollo_udp_bridge_receiver_component.cc
    src/apollo_ros_converter.cc
    src/apollo_ros_bridge.cc    
    ${OSMP_srcs})
  
  target_link_libraries(${_target}
    PUBLIC
      apollo_protos
      sensorview_grpc_proto
      open_simulation_interface_static
      ${PROJECT_SOURCE_DIR}/lib/libhdmap_common.so
      ${PROJECT_SOURCE_DIR}/lib/libutils.so
      ${PROJECT_SOURCE_DIR}/lib/libprotos.so
  )
endforeach()

# Test
add_executable(osmp_test 
  ${OSMP_srcs}
  src/osmp_test.cpp
)
target_link_libraries(osmp_test
  sensorview_grpc_proto
  open_simulation_interface_static
)

## OSMP(FMI2.0)
string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(src/OSMPDummySensor/modelDescription.in.xml modelDescription.xml @ONLY)
configure_file(src/OSMPDummySensor/OSMPDummySensorConfig.in.h OSMPDummySensorConfig.h)

add_library(OSMPDummySensor ${OSMP_srcs})
set_target_properties(OSMPDummySensor PROPERTIES PREFIX "")
target_compile_definitions(OSMPDummySensor PRIVATE "FMU_SHARED_OBJECT")
target_link_libraries(OSMPDummySensor open_simulation_interface_static protobuf::libprotobuf)

if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "win64")
	else()
		set(FMI_BINARIES_PLATFORM "win32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "linux64")
	else()
		set(FMI_BINARIES_PLATFORM "linux32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "darwin64")
	else()
		set(FMI_BINARIES_PLATFORM "darwin32")
	endif()
endif()

add_custom_command(TARGET OSMPDummySensor
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/OSMPDummySensor/OSMPDummySensor.cpp" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/OSMPDummySensor/OSMPDummySensor.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/OSMPDummySensorConfig.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/OSMPDummySensorConfig.h"
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OSMPDummySensor> $<$<PLATFORM_ID:Windows>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:OSMPDummySensor>>> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../OSMPDummySensor.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")


# Usage for Python
pybind11_add_module(osi_bridge
  src/libwrapper.cpp
  src/osi_client.cc
  src/osi_server.cc
  src/task/task_manager.cpp
  src/utils/sensor_data_osi_converter.cpp
  src/keti_ros_bridge.cc
  src/keti_ros_converter.cc
  src/morai_ros_bridge.cc
  ${OSMP_srcs}
)

target_link_libraries(osi_bridge
  PUBLIC
    ${_GRPC_GRPCPP}
    sensorview_grpc_proto
    open_simulation_interface_static
    ${PROJECT_SOURCE_DIR}/lib/libhdmap_common.so
    ${PROJECT_SOURCE_DIR}/lib/libutils.so
    ${PROJECT_SOURCE_DIR}/lib/libprotos.so
)

# Usage for Csharp
add_library(client SHARED
  src/libwrapper_csharp.cpp
  src/osi_client.cc
  src/task/task_manager.cpp
  src/utils/sensor_data_osi_converter.cpp
  src/keti_ros_bridge.cc
  src/keti_ros_converter.cc
  src/morai_ros_bridge.cc
  src/osi_server.cc
  ${OSMP_srcs}
)
target_link_libraries(client
  ${catkin_LIBRARIES}
  ${_GRPC_GRPCPP}
  sensorview_grpc_proto
  open_simulation_interface_static
  ${PROJECT_SOURCE_DIR}/lib/libhdmap_common.so
  ${PROJECT_SOURCE_DIR}/lib/libutils.so
  ${PROJECT_SOURCE_DIR}/lib/libprotos.so
)