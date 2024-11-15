#pragma once

#include "apollo_ros_converter.h"
#include "osi_bridge.h"


class ApolloROSBridge : public OSIBridge
{
public:
  // explicit ApolloROSBridge(std::string ip_address);
  explicit ApolloROSBridge(std::string client_ip_address, std::string server_ip_address);
  virtual ~ApolloROSBridge() = default;
  void StartBridge();
  void Stop();
  void ConvertThread();
  void PublishThread();
  template <typename T>
  bool SendData(const T &proto_msg,
                  const std::string proto_msg_name,
                  const std::string remote_ip,
                  const uint16_t remote_port);
  struct MsgResult
  {
    // std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t, CompressedImage>>>> camera_res_table;
    std::queue<std::future<std::pair<size_t, GnssBestPose>>> gnss_best_pose_res_table;
    std::queue<std::future<std::pair<size_t, Imu>>> imu_res_table;
    std::queue<std::future<std::pair<size_t, CorrectedImu>>> corimu_res_table;
    std::queue<std::future<std::pair<size_t, Gps>>> odom_res_table;
    std::queue<std::future<std::pair<size_t, Chassis>>> chassis_res_table;
    std::queue<std::future<std::pair<size_t, InsStat>>> ins_res_table;
    std::queue<std::future<std::pair<size_t, TrafficLightDetection>>> traffic_light_res_table;
    std::queue<std::future<std::pair<size_t, PerceptionObstacles>>> perception_obs_res_table;
  };

private:
  // ros
  ros::NodeHandle nh_;
  int num_of_camera_, num_of_lidar_;
  ros::Subscriber sub_is_changed_offset_;
  double x_offset_, y_offset_;

  // converting
  MsgResult msg_result_;
  ApolloROSConverter converter_;

  bool is_initialized_ = false;
  bool is_apollo_cmd_sub_initialized_ = false;
  bool is_sensor_view_buf_empty_ = true;
  bool is_apollo_cmd_ctrl_buf_empty_ = true;


  UDPBridgeReceiverComponent<apollo::control::ControlCommand> sub_ctrl_cmd_;
  
};