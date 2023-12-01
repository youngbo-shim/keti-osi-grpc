#pragma once

#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/support/async_stream.h>

// Sensor Error(Gausian) Modeling
#include <random>

#include "sensorview_rpc.grpc.pb.h"

using grpc::Channel;
using grpc::ClientAsyncReader;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using namespace osi3;

class OSIClient
{
public:
  OSIClient(std::queue<SensorView> *sensor_view_buf, std::mutex *sensor_view_mutex, std::string ip_address);

  ~OSIClient()
  {
    void *ignoredTag = nullptr;
    bool ok = false;
    while (cq_.Next(&ignoredTag, &ok));

    call_->reader->Finish(&call_->status, call_);
  }

  enum class CallState {
    NewCall,
    SendingRequest,
    ReceivingData,
    CallComplete
  };

  void StartListen();

  void handleNewCallState();
  void handleSendingRequestState();
  void handleReceivingDataState();
  void handleCallCompleteState();

  bool onNext(bool ok);

  void AsyncCompleteRpc();

private:
  struct AsyncClientCall
  {
    Request request;

    // Container for the data we expect from the server.
    SensorView reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // Storage for the status of the RPC upon completion.
    Status status;

    // std::unique_ptr<ClientAsyncReader<SensorView>> reader;
    std::unique_ptr<ClientAsyncReader<SensorView>> reader;
  };

private:
  AsyncClientCall *call_;
  std::unique_ptr<SensorViewRPC::Stub> stub_;
  std::string ip_address_;
  CompletionQueue cq_;
  CallState state_;
  std::queue<SensorView> *sensor_view_buf_;
  std::mutex *sensor_view_mutex_;

  std::thread client_thread_;
};