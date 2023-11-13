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

#include <ros/ros.h>
#include <ros/package.h>

#include "sensorview_rpc.grpc.pb.h"

using grpc::Channel;
using grpc::ClientAsyncReader;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using namespace keti::hdmap;

class OSIClient
{
public:
  OSIClient(std::queue<SensorView> *sensor_view_buf, std::mutex *sensor_view_mutex, std::string ip_address)
      : state_(CallState::NewCall), ip_address_(ip_address),
        sensor_view_buf_(sensor_view_buf), sensor_view_mutex_(sensor_view_mutex)
  {

    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
    stub_ = SensorViewRPC::NewStub(grpc::CreateCustomChannel(ip_address, grpc::InsecureChannelCredentials(), args));

    this->onNext(true);    
  }

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

  void StartListen(){
    client_thread_ = std::thread(&OSIClient::AsyncCompleteRpc, this);
  }

  void handleNewCallState() {
    call_ = new AsyncClientCall;
    call_->request.set_host_name(ip_address_);

    call_->reader = stub_->PrepareAsyncGetSensorView(&call_->context, call_->request, &cq_);

    // StartCall initiates the RPC call
    state_ = CallState::SendingRequest;
    call_->reader->StartCall(call_);
  }

  void handleSendingRequestState() {
    state_ = CallState::ReceivingData;

    call_->reader->Read(&call_->reply, call_);    
  }

  void handleReceivingDataState() {
    static size_t num_data = 0;
    const auto &sensor_view = call_->reply;

    std::chrono::time_point<std::chrono::system_clock> now =
          std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto current_nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;

    double sensor_view_stamp = sensor_view.timestamp().seconds()+sensor_view.timestamp().nanos()*1e-9;
    double communication_time = current_nanoseconds - sensor_view_stamp;
    // std::cout.precision(3);
    // std::cout << std::fixed << "[sensor, current] : " << sensor_view_stamp << ", " << current_nanoseconds << std::endl;
    // std::cout << "[msg size, communication speed] : " << sensor_view.ByteSizeLong() << ", " << (8.*sensor_view.ByteSizeLong()/(1024*1024))/communication_time << "Mbps" << std::endl;
    
    {
      std::lock_guard<std::mutex> lock(*sensor_view_mutex_);
      std::cout << "start push sensor view" << std::endl;
      sensor_view_buf_->push(sensor_view);
      std::cout << "finish push sensor view" << std::endl;
    }

    call_->reply.Clear();
    call_->reader->Read(&call_->reply, call_);

  }

  void handleCallCompleteState()
  {
    switch (call_->status.error_code())
    {
    case grpc::OK:
      std::cout << "[" << call_->request.host_name() << "]: RPC completed" << std::endl;
      break;

    case grpc::CANCELLED:
      std::cout << "[" << call_->request.host_name() << "]: RPC cancelled" << std::endl;
      break;

    default:
      std::cout << "[" << call_->request.host_name() << "]: RPC failed: " << call_->status.error_message() << std::endl;
      break;
    }
  }

  bool onNext(bool ok)
  {
    try
    {
      if (ok)
      {
        if (state_ == CallState::NewCall)
        {
          std::cout << "NewCall" << std::endl;
          this->handleNewCallState();
        }
        else if (state_ == CallState::SendingRequest)
        {
          std::cout << "SendingRequest" << std::endl;
          this->handleSendingRequestState();
        }
        else if (state_ == CallState::ReceivingData)
        {
          this->handleReceivingDataState();
        }
        else if (state_ == CallState::CallComplete)
        {
          std::cout << "CallComplete" << std::endl;
          this->handleCallCompleteState();
          return false;
        }
      }
      else
      {
        state_ = CallState::CallComplete;
        call_->reader->Finish(&call_->status, call_);
      }
    }
    catch (std::exception &e)
    {
      gpr_log(GPR_ERROR, "Processing error: %s", e.what());
    }
    catch (...)
    {
      gpr_log(GPR_ERROR, "Processing error: unknown exception caught");
    }

    return true;
  }

  void AsyncCompleteRpc()
  {
    void *got_tag;
    bool ok = false;

    // Block until the next result is available in the completion queue "cq".
    while (1)
    {
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      cq_.Next(&got_tag, &ok);
      // std::cout << "cq_Next()" << std::endl;

      auto duration = now.time_since_epoch();
      auto micros =
          std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
      // std::cout << "[" << micros << "], Next!!" << std::endl;
      AsyncClientCall *call = static_cast<AsyncClientCall *>(got_tag);
      // std::cout << "ok : " << ok << std::endl;
      if(!ok) continue;
      if (!this->onNext(ok))
      {
        break;
      }
    }
  }

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