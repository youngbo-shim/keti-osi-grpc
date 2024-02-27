#include "osi_server.h"

bool OSICallData::Proceed(bool ok) {
  if (ok || !has_data_) {
    std::chrono::time_point<std::chrono::system_clock> start =
                                      std::chrono::system_clock::now();
    if (status_ == CREATE)
    {
      std::cout << "CREATE(" << this << ")" << std::endl;
      status_ = PROCESS;
      service_->RequestGetSensorView(&ctx_, &request_, &responder_, cq_, cq_, this);
    }
    else if (status_ == PROCESS)
    {
      reply_.Clear();          
      has_data_ = false;

      if(!server_sensor_view_buf_->empty()){
        reply_ = server_sensor_view_buf_->front();
        // std::cout << "server streaming, server_sensor_view_buf_->size() : " << server_sensor_view_buf_->size() << std::endl;
        server_sensor_view_buf_->pop();
        has_data_ = true;
      }

      if (has_data_){
        std::chrono::time_point<std::chrono::system_clock> now =
              std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto nanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;
        unsigned int seconds = nanoseconds;
        auto current_nanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;

        reply_.mutable_timestamp()->set_seconds(seconds);
        reply_.mutable_timestamp()->set_nanos((nanoseconds-seconds)*1e9);

        responder_.Write(reply_, this);
        
        std::chrono::duration<double> running_time = std::chrono::system_clock::now() - start;
        std::cout.precision(3);
        // std::cout << "converting running time : " << running_time.count() * 1000 << "ms" << std::endl;
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

// There is no shutdown handling in this code.
void OSIServer::Run()
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
  builder.SetMaxSendMessageSize(1 * 1024 * 1024 * 1024);
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;
}

// This can be run in multiple threads if needed.
void OSIServer::HandleRpcs()
{
  // Spawn a new CallData instance to serve new clients.
  new OSICallData(&service_, cq_.get(), server_sensor_view_buf_);
  void *tag; // uniquely identifies a request.
  bool ok;

  while (true)
  {
    if(server_sensor_view_buf_->empty()){
      std::this_thread::sleep_for( std::chrono::milliseconds(1)); // while loop trigger
      continue;
    }
    cq_->Next(&tag, &ok);
    // std::cout << "next" << std::endl;

    // GPR_ASSERT(ok);
    if (!ok) continue;

    if ( !static_cast<OSICallData *>(tag)->Proceed(ok)){
      break;
    }
  }

  std::cout << "thread join" << std::endl;
}