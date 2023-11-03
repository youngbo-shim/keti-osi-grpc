#include <iostream>
#include <thread>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/support/async_stream.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>

#include "sensorview_rpc.grpc.pb.h"

using grpc::Channel;
using grpc::ClientAsyncReader;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using osi3::Request;
using osi3::SensorView;
using osi3::SensorViewRPC;

// image_transport::Publisher pub_img;
ros::Publisher pub_img;

class SensorViewClient
{
public:
  SensorViewClient(std::shared_ptr<Channel> channel)
      : stub_(SensorViewRPC::NewStub(channel)), state_(CallState::NewCall)
  {

    this->onNext(true);
  }

  ~SensorViewClient()
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

  void handleNewCallState() {
    call_ = new AsyncClientCall;
    call_->request.set_host_name("localhost:50051");

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
    double time_diff = current_nanoseconds - sensor_view_stamp;
    std::cout << "[sensor, current] : " << sensor_view_stamp << ", " << current_nanoseconds << std::endl;
    std::cout << "[communication speed] : " << (8.*sensor_view.ByteSizeLong()/(1024*1024))/time_diff << "Mbps" << std::endl;

    if (sensor_view.camera_sensor_view_size() > 0)
    {
      std::cout << "RPC Success : size(" << sensor_view.camera_sensor_view()[0].image_data().size() << ")" << std::endl;
      // if (sensor_view.camera_sensor_view_size() > 0)
      // {
      //   std::cout << (int)sensor_view.camera_sensor_view()[0].image_data()[3] << std::endl;
      // }
  
      sensor_msgs::CompressedImage img;
      img.header.stamp = ros::Time::now();
      img.format = "png";
      img.data = {sensor_view.camera_sensor_view()[0].image_data().begin(),
                  sensor_view.camera_sensor_view()[0].image_data().end()};

      pub_img.publish(img);

      call_->reply.Clear();
      call_->reader->Read(&call_->reply, call_);
    }
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
    while (cq_.Next(&got_tag, &ok))
    {
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      auto micros =
          std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
      std::cout << "[" << micros << "], Next!!" << std::endl;
      AsyncClientCall *call = static_cast<AsyncClientCall *>(got_tag);      
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
  CompletionQueue cq_;
  CallState state_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "grpc_client");
  ros::NodeHandle nh;
  
  // image_transport::ImageTransport it(nh);
  // pub_img = it.advertise("/grpc/compressed_image",1);
  pub_img = nh.advertise<sensor_msgs::CompressedImage>("/grpc/compressed",1);

  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
  SensorViewClient client(
      grpc::CreateCustomChannel("localhost:50051",
                                grpc::InsecureChannelCredentials(),
                                args));

  std::thread thread_ = std::thread(&SensorViewClient::AsyncCompleteRpc, &client);

  thread_.join();

  return 0;
}