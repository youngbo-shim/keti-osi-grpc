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

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <morai_msgs/ObjectStatusList.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensorview_rpc.grpc.pb.h"
#include "task/task.h"
#include "osi_bridge.cc"


using google::protobuf::RepeatedPtrField;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;
using osi3::HostVehicleData;

class AutowareROSConverter
{
public:
  AutowareROSConverter();
  ~AutowareROSConverter() = default;

  std::pair<size_t,sensor_msgs::PointCloud2> ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                                                       int lidar_id, size_t seq);
  std::pair<size_t,sensor_msgs::CompressedImage> ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                            int camera_id, size_t seq);
  std::pair<size_t,autoware_msgs::DetectedObjectArray> ProcObj(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                       std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                       size_t seq);
  std::pair<size_t,sensor_msgs::Imu> ProcImu(std::shared_ptr<HostVehicleData>& imu_sensor_view, size_t imu_seq);
  std::pair<size_t, geometry_msgs::PoseStamped> ProcEgoVehicleState(std::shared_ptr<HostVehicleData>& ego_vehicle_state_view, size_t ego_vehicle_state_seq);
  std::pair<size_t, geometry_msgs::TwistStamped> ProcEgoVehicleSpeed(std::shared_ptr<HostVehicleData>& ego_vehicle_state_view, size_t ego_vehicle_state_seq);
  std::pair<size_t, morai_msgs::GPSMessage> ProcGps(std::shared_ptr<HostVehicleData>& gps_sensor_view, size_t gps_seq);
  std_msgs::ColorRGBA colorCategory10(int i);
  void SetoffsetX(double x_offset){ x_offset_ = x_offset; }
  void SetoffsetY(double y_offset){ y_offset_ = y_offset; }
  double GetoffsetX(){return x_offset_; }
  double GetoffsetY(){ return y_offset_; }

private:
  double x_offset_, y_offset_;

};