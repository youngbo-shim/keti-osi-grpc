#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/alarm.h>

#include "sensorview_rpc.grpc.pb.h"

using grpc::Server;
using grpc::ServerAsyncWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using osi3::Request;
using osi3::SensorView;
using osi3::SensorViewRPC;
using osi3::MountingPosition;
using osi3::HostVehicleData;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::RadarSensorView;
using osi3::GroundTruth;

// Class encompasing the state and logic needed to serve a request.
class OSICallData
{
public:
  // Take in the "service" instance (in this case representing an asynchronous
  // server) and the completion queue "cq" used for asynchronous communication
  // with the gRPC runtime.
  OSICallData(SensorViewRPC::AsyncService *service, ServerCompletionQueue *cq,
              std::queue<SensorView> *server_sensor_view_buf)
      : service_(service), cq_(cq), server_sensor_view_buf_(server_sensor_view_buf), responder_(&ctx_), status_(CREATE)
  {
    num_writing_ = 0;
    // Invoke the serving logic right away.
    Proceed(true);
  }

  bool Proceed(bool ok);

private:
  // The means of communication with the gRPC runtime for an asynchronous
  // server.
  SensorViewRPC::AsyncService *service_;
  // The producer-consumer queue where for asynchronous server notifications.
  ServerCompletionQueue *cq_;
  // Context for the rpc, allowing to tweak aspects of it such as the use
  // of compression, authentication, as well as to send metadata back to the
  // client.
  ServerContext ctx_;

  // What we get from the client.
  Request request_;
  // What we send back to the client.
  SensorView reply_;

  // matching table
  // TypeTable morai_to_osi_matching_table_tl_;
  // ObjTable morai_to_osi_matching_table_obj_;
  // std::unordered_map<std::string, std::vector<std::string>> id_table_;

  // Injected data
  std::queue<SensorView> *server_sensor_view_buf_;

  // The means to get back to the client.
  ServerAsyncWriter<SensorView> responder_;

  // Let's implement a tiny state machine with the following states.
  enum CallStatus
  {
    CREATE,
    PROCESS,
    FINISH
  };
  CallStatus status_; // The current serving state.

  size_t num_writing_;
  bool has_data_= true;
};

class OSIServer final
{
public:
  explicit OSIServer(std::queue<SensorView> *server_sensor_view_buf, std::string address) 
    : server_sensor_view_buf_(server_sensor_view_buf), address_(address) {}
  
  ~OSIServer(){
    server_->Shutdown();
    cq_->Shutdown();
  }

  // There is no shutdown handling in this code.
  void Run();
  // This can be run in multiple threads if needed.
  void HandleRpcs();

private:
  std::unique_ptr<ServerCompletionQueue> cq_;
  SensorViewRPC::AsyncService service_;
  std::unique_ptr<Server> server_;
  std::string address_;

  // Ground Truth
  std::queue<SensorView> *server_sensor_view_buf_;

};