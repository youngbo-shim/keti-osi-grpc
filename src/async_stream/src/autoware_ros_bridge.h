#pragma once

#include "autoware_ros_converter.h"
#include "osi_bridge.h"

class AutowareROSBridge : public OSIBridge
{
public:
  explicit AutowareROSBridge(std::string client_ip_address, std::string server_ip_address);
  virtual ~AutowareROSBridge() = default;
  void StartBridge();
  void Stop();
  void ConvertThread();
  void PublishThread();

  struct MsgResult
  {
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table;
    std::queue<std::future<std::pair<size_t,autoware_msgs::DetectedObjectArray>>> obj_res_table;
    std::queue<std::future<std::pair<size_t,morai_msgs::GPSMessage>>> gps_res_table;
    std::queue<std::future<std::pair<size_t,sensor_msgs::Imu>>> imu_res_table;
    std::queue<std::future<std::pair<size_t,geometry_msgs::PoseStamped>>> ego_state_res_table;
    std::queue<std::future<std::pair<size_t,geometry_msgs::TwistStamped>>> ego_speed_res_table;    
  };

private:
  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> pub_imgs_, pub_clouds_;
  ros::Publisher pub_tls_, pub_objs_, pub_imu_, pub_gps_, pub_morai_ego_state_, 
                 pub_autoware_ego_state_, pub_autoware_ego_speed_, pub_autoware_objects_;
  int num_of_camera_, num_of_lidar_;
  ros::Subscriber sub_is_changed_offset_;
  double x_offset_, y_offset_;

  // converting
  MsgResult msg_result_;
  AutowareROSConverter converter_;

  bool is_initialized_ = false;
};