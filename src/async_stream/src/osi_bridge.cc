#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "sensorview_rpc.grpc.pb.h"
#include "hdmap.h"
#include "osi_client.cc"

using namespace keti::hdmap;

static constexpr unsigned int Hash(const char* str){
  return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * Hash(str + 1) : 8603;
}

class OSIBridge
{
public:
  explicit OSIBridge(std::string ip_address)
  {
    osi_client_ = new OSIClient(&sensor_view_buf_, &sensor_view_mutex_, ip_address);
  }

  virtual ~OSIBridge() = default;

  void ClientStartListen(){
    osi_client_->StartListen();
  }

  virtual void StartBridge() = 0;

  virtual void Stop() = 0;

protected:
  // osi
  std::queue<SensorView> sensor_view_buf_;
  std::mutex sensor_view_mutex_;
  OSIClient *osi_client_;

  // thread
  std::thread converting_thread_;
  std::thread publish_thread_;
};