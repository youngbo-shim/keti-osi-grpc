#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>

#include "sensorview_rpc.grpc.pb.h"
#include "task/task.h"
#include "osi_bridge.h"
#include "keti_ros_converter.h"
#include "control_msgs/VehicleCMD.h"

#include "OSMPDummySensor.h"

class KetiROSBridge : public OSIBridge
{
public:
  struct MsgResult
  {
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table; // seq: 디버깅용 -> 빼도 상관없음
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,radar_msgs::RadarScan>>>> radar_res_table;
    std::queue<std::future<std::pair<size_t,perception_msgs::TrafficLights>>> tl_res_table;
    std::queue<std::future<std::pair<size_t,cyber_perception_msgs::PerceptionObstacles>>> obj_res_table;
    std::queue<std::future<std::tuple<size_t,control_msgs::VehicleState,geometry_msgs::PoseStamped,geometry_msgs::TransformStamped>>> ego_state_res_table;
  };

  explicit KetiROSBridge(std::string client_ip_address, std::string server_ip_address);
  virtual ~KetiROSBridge() = default;

  void StartBridge();
  void Stop();

  void CallbackUpdateOffsetParams(const std_msgs::Bool& check);
  void CallbackKetiCmd(const control_msgs::VehicleCMD& msg);

  void ConvertThread();
  void PublishThread();

private:
  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> pub_imgs_, pub_clouds_, pub_radar_;
  ros::Publisher pub_tls_, pub_objs_, pub_vehicle_state_, pub_current_pose_;
  int num_of_camera_, num_of_lidar_, num_of_radar_;
  ros::Subscriber sub_is_changed_offset_, sub_keti_cmd_;

  // hdmap
  std::shared_ptr<HDMap> hdmap_;

  // converting
  KetiROSConverter converter_;
  MsgResult msg_result_;

  bool is_initialized_ = false;
};