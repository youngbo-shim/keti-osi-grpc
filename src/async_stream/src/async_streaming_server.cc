#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/alarm.h>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include "sensorview_rpc.grpc.pb.h"


using grpc::Server;
// using grpc::ServerAsyncResponseWriter;
using grpc::ServerAsyncWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using osi3::Request;
using osi3::SensorView;
using osi3::SensorViewRPC;

class SensorViewRPCImpl final
{
public:
  explicit SensorViewRPCImpl(std::string address) : address_(address) {    
    morai_camera_sub_ = nh_.subscribe("/image_jpeg/compressed",
                                      10,
                                      &SensorViewRPCImpl::CallbackImage,
                                      this);
  }

  ~SensorViewRPCImpl()
  {
    server_->Shutdown();
    cq_->Shutdown();
  }

  // There is no shutdown handling in this code.
  void Run()
  {
    std::string server_address(address_.c_str());

    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service_" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *asynchronous* service.
    builder.RegisterService(&service_);
    // Get hold of the completion queue used for the asynchronous communication
    // with the gRPC runtime.
    cq_ = builder.AddCompletionQueue();
    // Finally assemble the server.
    builder.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
    server_ = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;
  }

  // This can be run in multiple threads if needed.
  void HandleRpcs()
  {
    // Spawn a new CallData instance to serve new clients.
    new CallData(&service_, cq_.get(), &img_buf_);
    void *tag; // uniquely identifies a request.
    bool ok;

    while (true)
    {
      GPR_ASSERT(cq_->Next(&tag, &ok));
      std::cout << "[tag, ok] : " << tag << ", " << ok << std::endl;
      if ( !static_cast<CallData *>(tag)->Proceed(ok)){
        break;
      }
      // if (!static_cast<CallData *>(tag)->Proceed(ok))
      //   break;
    }

    std::cout << "thread join" << std::endl;
  }

  void CallbackImage( const sensor_msgs::CompressedImageConstPtr& img ){
    img_buf_.push_back(img);
  }

private:
  // Class encompasing the state and logic needed to serve a request.
  class CallData
  {
  public:
    // Take in the "service" instance (in this case representing an asynchronous
    // server) and the completion queue "cq" used for asynchronous communication
    // with the gRPC runtime.
    CallData(SensorViewRPC::AsyncService *service, ServerCompletionQueue *cq,
            std::list<sensor_msgs::CompressedImageConstPtr> *img_buf)
        : service_(service), cq_(cq), img_buf_(img_buf), responder_(&ctx_), status_(CREATE)
    {
      num_writing_ = 0;

      // Invoke the serving logic right away.
      Proceed(true);
    }

    bool Proceed(bool ok)
    {
      if (ok || !has_data_) {
        if (status_ == CREATE)
        {
          std::cout << "CREATE(" << this << ")" << std::endl;
          status_ = PROCESS;
          service_->RequestGetSensorView(&ctx_, &request_, &responder_, cq_, cq_, this);

          img_buf_->clear();
        }
        else if (status_ == PROCESS)
        {
          if ( img_buf_->size() > 0 ){

            // The actual processing.
            reply_.Clear();

            std::cout << "PROCESS, Img buffer size : " << img_buf_->size() << std::endl;

            std::chrono::time_point<std::chrono::system_clock> now =
                  std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto nanoseconds =
                std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;
            unsigned int seconds = nanoseconds;

            reply_.mutable_timestamp()->set_seconds(seconds);
            reply_.mutable_timestamp()->set_nanos((nanoseconds-seconds)*1e9);
            // const size_t width = 1920, height = 1080, rgb_size = 3;
            // auto camera_view = reply_.add_camera_sensor_view();
            // auto img_buffer = camera_view->mutable_image_data();
            // img_buffer->resize(width * height * rgb_size);
            // for (int i = 0; i < (width * height * rgb_size); i++)
            // {
            //   (*img_buffer)[i] = static_cast<char>(i);
            // }            

            auto& sending_img = img_buf_->front();

            auto camera_view = reply_.add_camera_sensor_view();
            camera_view->mutable_image_data()->resize(sending_img->data.size());
            *camera_view->mutable_image_data() = {sending_img->data.begin(), sending_img->data.end()};

            std::cout << "img size : " << sending_img->data.size() << std::endl;

            responder_.Write(reply_, this);

            img_buf_->pop_front();            

            // num_writing_++;            
            
            // if (num_writing_ > 100)
            // {
            //   new CallData(service_, cq_, img_buf_);

            //   delete this;
            //   return false;
            // }

            has_data_ = true;
          }
          else {
            std::cout << "There is no data in image buffer." << std::endl;
            grpc::Alarm alarm;
            alarm.Set(cq_, gpr_now(gpr_clock_type::GPR_CLOCK_REALTIME), this);

            has_data_ = false;
          }
          
          status_ = PROCESS;
        }
      }
      else
      {
        std::cout << "FINISH" << std::endl;
        status_ = FINISH;        

        responder_.Finish(Status::OK, this);

        delete this;
        return false;
      }

      return true;
    }

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

    // Injected data
    std::list<sensor_msgs::CompressedImageConstPtr> *img_buf_;

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

private:
  std::unique_ptr<ServerCompletionQueue> cq_;
  SensorViewRPC::AsyncService service_;
  std::unique_ptr<Server> server_;
  std::string address_;

  // ros
  ros::NodeHandle nh_;
  ros::Subscriber morai_camera_sub_;

  std::list<sensor_msgs::CompressedImageConstPtr> img_buf_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "morai_keti_bridge_grpc");

  SensorViewRPCImpl server("localhost:50051");
  server.Run();

  std::thread thread(&SensorViewRPCImpl::HandleRpcs, &server);

  ros::spin();
  thread.join();

  return 0;
}
