cmake_minimum_required(VERSION 3.5.1)
project(async_stream)


# set c++ compile config
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC") # changed

include(cmake/common.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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
  control_msgs
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
    ${OSMP_srcs})
  
  target_link_libraries(${_target}
    PUBLIC
      sensorview_grpc_proto
      open_simulation_interface_static
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

##  Python bind
pybind11_add_module(osi_bridge
  src/libwrapper.cpp
  src/osi_client.cc
  src/osi_server.cc
  src/task/task_manager.cpp
  src/utils/sensor_data_osi_converter.cpp
  src/keti_ros_bridge.cc
  src/keti_ros_converter.cc
)

target_link_libraries(osi_bridge
  PUBLIC
    ${_GRPC_GRPCPP}
    sensorview_grpc_proto
    open_simulation_interface_static
)  

# Python wrapper with pybind11
# set(wrapper_srcs
#   src/libwrapper.cpp
# )

# add_library(client_lib ${wrapper_srcs})