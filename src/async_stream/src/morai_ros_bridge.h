#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>

#include "sensorview_rpc.grpc.pb.h"
#include "osi_bridge.h"

class MoraiROSBridge : public OSIBridge
{
public:
  explicit MoraiROSBridge(std::string client_ip_address, std::string server_ip_address);
  virtual ~MoraiROSBridge() = default;

  void StartBridge();
  void Stop();

  void PublishThread();

private:
  // ros
  ros::NodeHandle nh_;
  ros::Publisher pub_morai_cmd_;

  bool is_initialized_ = false;
  std::string bridge_name_;
};