#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "sensorview_rpc.grpc.pb.h"
#include "hdmap.h"
#include "osi_client.h"
#include "osi_server.h"

using namespace keti::hdmap;

class OSIBridge
{
public:
  explicit OSIBridge(std::string client_ip_address, std::string server_ip_address)
  {
    osi_client_ = new OSIClient(&sensor_view_buf_, &sensor_view_mutex_, client_ip_address);
    osi_server_ = new OSIServer(&server_sensor_view_buf_, server_ip_address);
  }

  virtual ~OSIBridge() = default;

  void ClientStartListen(){
    osi_client_->StartListen();
  }

  void ServerStartStream(){
    osi_server_->Run();
    server_streaming_thread_ = std::thread(&OSIServer::HandleRpcs, osi_server_);
  }

  virtual void StartBridge() = 0;

  virtual void Stop() = 0;

protected:
  // osi
  std::queue<SensorView> sensor_view_buf_;
  std::mutex sensor_view_mutex_;
  OSIClient *osi_client_;

  std::queue<SensorView> server_sensor_view_buf_;
  OSIServer *osi_server_;

  // thread
  std::thread converting_thread_;
  std::thread publish_thread_;
  std::thread server_streaming_thread_;
};