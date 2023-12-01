#pragma once
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <radar_msgs/RadarScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include "sensorview_rpc.grpc.pb.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/TrafficLights.h"
#include "perception_msgs/TrafficLight.h"
#include "perception_msgs/TrafficSignalPhase.h"
#include "control_msgs/VehicleState.h"

#include "hdmap.h"
#include "math/math_utils.h"

#define SPEED_OF_LIGHT 299792458

using google::protobuf::RepeatedPtrField;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::RadarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;
using osi3::HostVehicleData;

using namespace keti::common;
using namespace keti::hdmap;

class KetiROSConverter
{
public:
  KetiROSConverter();
  ~KetiROSConverter() = default;

  void TrafficLightIdMathching();
  void TrafficLightOSIToKetiMatching();

  std::pair<size_t,sensor_msgs::PointCloud2> ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                                                       int lidar_id, size_t seq);

  std::pair<size_t,sensor_msgs::CompressedImage> ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                            int camera_id, size_t seq); 

  std::pair<size_t, radar_msgs::RadarScan> ProcRadar(std::shared_ptr<RadarSensorView> radar_sensor_view,
                                                     int radar_id, size_t seq);

  int CalculatePhsaeCode(std::vector<osi3::TrafficLight> osi_tls);

  std::pair<size_t,perception_msgs::TrafficLights> ProcTL(std::shared_ptr<RepeatedPtrField<osi3::TrafficLight>> osi_tls,
                                                          std::shared_ptr<HDMap> hdmap,
                                                          size_t seq);

  std::pair<size_t,cyber_perception_msgs::PerceptionObstacles> ProcObj(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                       std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                       size_t seq);

  std::tuple<size_t,control_msgs::VehicleState,geometry_msgs::PoseStamped,geometry_msgs::TransformStamped> ProcEgoVehicleState(std::shared_ptr<HostVehicleData> host_vehicle_data,
                                                                                                                               size_t seq, std::shared_ptr<HDMap> hdmap); 

private:

  std::unordered_map<std::string, std::vector<std::string>> tl_id_match_table_;
  std::unordered_map<int, perception_msgs::TrafficSignalPhase> tl_phase_matching_table_;

};