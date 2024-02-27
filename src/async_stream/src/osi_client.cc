#include "osi_client.h"

OSIClient::OSIClient(std::queue<SensorView> *sensor_view_buf, std::mutex *sensor_view_mutex, std::string ip_address)
      : state_(CallState::NewCall), ip_address_(ip_address),
        sensor_view_buf_(sensor_view_buf), sensor_view_mutex_(sensor_view_mutex)
{

  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
  auto channel = grpc::CreateCustomChannel(ip_address, grpc::InsecureChannelCredentials(), args);
  while(channel->GetState(true) != 2){
    std::cout << "channel is NOT READY! state : " << channel->GetState(true) << std::endl;
    std::this_thread::sleep_for( std::chrono::milliseconds(500));
  }
  // stub_ = SensorViewRPC::NewStub(grpc::CreateCustomChannel(ip_address, grpc::InsecureChannelCredentials(), args));
  stub_ = SensorViewRPC::NewStub(channel);

  this->onNext(true);    
}

void OSIClient::StartListen(){
  client_thread_ = std::thread(&OSIClient::AsyncCompleteRpc, this);
}

void OSIClient::handleNewCallState() {
  call_ = new AsyncClientCall;
  // call_->request.set_host_name(ip_address_);
  std::cout << "ip_address_ : " << ip_address_ << std::endl;

  call_->reader = stub_->PrepareAsyncGetSensorView(&call_->context, call_->request, &cq_);

  // StartCall initiates the RPC call
  state_ = CallState::SendingRequest;
  call_->reader->StartCall(call_);
}

void OSIClient::handleSendingRequestState() {
  state_ = CallState::ReceivingData;

  call_->reader->Read(&call_->reply, call_);    
}

void OSIClient::handleReceivingDataState() {  
  {
    std::lock_guard<std::mutex> lock(*sensor_view_mutex_);
    sensor_view_buf_->push(std::move(call_->reply));
  }

  call_->reply.Clear();
  call_->reader->Read(&call_->reply, call_);
}

void OSIClient::handleCallCompleteState()
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

bool OSIClient::onNext(bool ok)
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
        // std::cout << "ReceivingDataState" << std::endl;
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

void OSIClient::AsyncCompleteRpc()
{
  void *got_tag;
  bool ok = false;

  // Block until the next result is available in the completion queue "cq".
  while (1)
  {
    std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();
    // std::cout << "before next" << std::endl;
    cq_.Next(&got_tag, &ok);
    // auto status = cq_.AsyncNext(&got_tag, &ok, std::chrono::system_clock::now() + std::chrono::milliseconds(200));  // 1s timeout

    // if (status == grpc::CompletionQueue::SHUTDOWN) {
    //     // CompletionQueue가 종료된 경우
    //     std::cout << "SHUTDOWN" << std::endl;
    //     return;
    // } else if (status == grpc::CompletionQueue::TIMEOUT) {
    //     // Timeout이 발생한 경우
    //     std::cout << "TIMEOUT" << std::endl;
    //     continue;
    // }
    // std::cout << "after next : " << ok << std::endl;    
    AsyncClientCall *call = static_cast<AsyncClientCall *>(got_tag);
    // std::cout << "call_state : " << call->status.error_details() << ", " << ok << std::endl;    
    if(!ok) continue;

    if (!this->onNext(ok))
    {
      break;
    }
  }
}