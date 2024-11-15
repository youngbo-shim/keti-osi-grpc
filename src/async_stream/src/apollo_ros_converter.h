#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/TrafficLights.h"
#include "perception_msgs/TrafficLight.h"
#include "perception_msgs/TrafficSignalPhase.h"

#include "morai_msgs/EgoVehicleStatus.h"
#include "morai_msgs/GPSMessage.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/TrafficLights.h"
#include "perception_msgs/TrafficLight.h"
#include "perception_msgs/TrafficSignalPhase.h"
#include <tf/transform_listener.h>
#include "math/math_utils.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <morai_msgs/ObjectStatusList.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensorview_rpc.grpc.pb.h"
#include "apollo_gnss_best_pose.pb.h"
#include "apollo_imu.pb.h"
#include "apollo_ins.pb.h"
#include "apollo_gps.pb.h"
#include "apollo_corimu.pb.h"
#include "apollo_chassis.pb.h"
#include "apollo_traffic_light_detection.pb.h"
#include "apollo_perception_obstacle.pb.h"
#include "apollo_sensor_image.pb.h"
#include "apollo_velodyne.pb.h"
#include "task/task.h"

// apollo socket communication
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "utils/apollo_bridge_proto_serialized_buf.h"
#include "utils/apollo_udp_bridge_receiver_component.h"

#define MORAI_VEHICLE_MAX_STEER_ANGLE_IN_DEG 36.25
#define DRIVING_MANUAL_MODE 0
#define DRIVING_AUTO_MODE 1

using google::protobuf::RepeatedPtrField;
using namespace keti::common;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;
using osi3::HostVehicleData;

using apollo::drivers::gnss::GnssBestPose;
using apollo::drivers::gnss::Imu;
using apollo::drivers::gnss::InsStat;
using apollo::localization::CorrectedImu;
using apollo::localization::Gps;
using apollo::canbus::Chassis;
using apollo::perception::TrafficLightDetection;
using apollo::perception::PerceptionObstacles;
using apollo::drivers::CompressedImage;
using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::velodyne::VelodynePacket;

using apollo::bridge::BridgeProtoSerializedBuf;
using apollo::bridge::BridgeProtoDiserializedBuf;
using apollo::bridge::UDPBridgeReceiverComponent;

static const int BLOCKS_PER_PACKET = 12;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use cstdint types, so things work with both 64 and 32-bit machines
 */
struct RawBlock {
  uint16_t laser_block_id;  ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;        ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
};

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  status has either a temperature encoding or the microcode level
 */
struct RawPacket {
  RawBlock blocks[BLOCKS_PER_PACKET];
  // uint16_t revolution;
  // uint8_t status[PACKET_STATUS_SIZE];
  unsigned int gps_timestamp;
  unsigned char status_type;
  unsigned char status_value;
};

class ApolloROSConverter
{
public:
  ApolloROSConverter();
  ~ApolloROSConverter() = default;

  std::pair<size_t,geometry_msgs::PoseStamped> ProcEgoVehicleState(const HostVehicleData& ego_vehicle_state_view, size_t ego_vehicle_state_seq);
  std::pair<size_t,geometry_msgs::TwistStamped> ProcEgoVehicleSpeed(const HostVehicleData& ego_vehicle_state_view, size_t ego_vehicle_state_seq);
  std::pair<size_t,morai_msgs::GPSMessage> ProcGps(const HostVehicleData& gps_sensor_view, size_t gps_seq);
  std::pair<size_t,CompressedImage> ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                            int camera_id, size_t seq);
  std::pair<size_t,VelodyneScan> ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                                                       int lidar_id, size_t seq);
  std::pair<size_t,Chassis> ProcVehicleChassis(const HostVehicleData& chassis_sensor_view, size_t seq);
  std::pair<size_t,GnssBestPose> ProcBestPose(const HostVehicleData& gps_sensor_view, size_t seq);
  std::pair<size_t,Imu> ProcImu(const HostVehicleData& imu_sensor_view, size_t seq);
  std::pair<size_t,CorrectedImu> ProcCorImu(const HostVehicleData& corimu_sensor_view, size_t seq);
  std::pair<size_t,Gps> ProcOdometry(const HostVehicleData& odometry_sensor_view, size_t seq);
  std::pair<size_t,InsStat> ProcIns(size_t seq);
  std::pair<size_t,TrafficLightDetection> ProcTrafficLightDetection(const HostVehicleData& traffic_light_view, size_t seq);
  std::pair<size_t,PerceptionObstacles> ProcPerceptionObstacles(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                      std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                       size_t seq);
  std_msgs::ColorRGBA colorCategory10(int i);
  void SetoffsetX(const double x_offset){ x_offset_ = x_offset; }
  void SetoffsetY(const double y_offset){ y_offset_ = y_offset; }
  const double GetoffsetX(){return x_offset_; }
  const double GetoffsetY(){ return y_offset_; }

private:
  double x_offset_, y_offset_;
};