/*****************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "apollo_udp_bridge_receiver_component.h"

#include <ros/ros.h>
#include "apollo_macro.h"
#include "apollo_util.h"

namespace apollo {
namespace bridge {

template <typename T>
UDPBridgeReceiverComponent<T>::UDPBridgeReceiverComponent(){}

template <typename T>
UDPBridgeReceiverComponent<T>::~UDPBridgeReceiverComponent() {
  for (auto proto : proto_list_) {
    FREE_POINTER(proto);
  }
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::Init() {
  std::cout << "UDP bridge receiver init, starting..." << std::endl;
  bind_port_ = 15007;
  proto_name_ = "Control";
  topic_name_ = "/apollo/control";
  enable_timeout_ = false;
  std::cout << "UDP Bridge remote port is: " << bind_port_ << " (uint16_t)bind_port_): " << (uint16_t)bind_port_ << std::endl;
  std::cout << "UDP Bridge for Proto is: " << proto_name_ << std::endl;

  if (!InitSession((uint16_t)bind_port_)) {
    std::cout << "bind port fail" << std::endl;
    return false;
  }
  std::cout << "initialize session successful" << std::endl;
  MsgDispatcher();
  
  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::InitSession(uint16_t port) {
  std::cout << "InitSession port:  " << port << std::endl;
  return listener_->Initialize(this, &UDPBridgeReceiverComponent<T>::MsgHandle,
                               port);
}

template <typename T>
void UDPBridgeReceiverComponent<T>::MsgDispatcher() {
  std::cout << "msg dispatcher start successful" << std::endl;
  listener_->Listen();
}

template <typename T>
BridgeProtoDiserializedBuf<T>
    *UDPBridgeReceiverComponent<T>::CreateBridgeProtoBuf(
        const BridgeHeader &header) {
  if (IsTimeout(header.GetTimeStamp())) {
    typename std::vector<BridgeProtoDiserializedBuf<T> *>::iterator itor =
        proto_list_.begin();
    for (; itor != proto_list_.end();) {
      if ((*itor)->IsTheProto(header)) {
        BridgeProtoDiserializedBuf<T> *tmp = *itor;
        FREE_POINTER(tmp);
        itor = proto_list_.erase(itor);
        break;
      }
      ++itor;
    }
    return nullptr;
  }

  for (auto proto : proto_list_) {
    if (proto->IsTheProto(header)) {
      return proto;
    }
  }
  BridgeProtoDiserializedBuf<T> *proto_buf_ = new BridgeProtoDiserializedBuf<T>;
  if (!proto_buf_) {
    return nullptr;
  }
  proto_buf_->Initialize(header);
  proto_list_.push_back(proto_buf_);
  return proto_buf_;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::IsProtoExist(const BridgeHeader &header) {
  for (auto proto : proto_list_) {
    if (proto->IsTheProto(header)) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::IsTimeout(double time_stamp) {
  if (enable_timeout_ == false) {
    return false;
  }
  double cur_time = ros::Time::now().toSec();
  double FLAGS_timeout = 1.0;
  // double cur_time = apollo::cyber::Clock::NowInSeconds();
  if (cur_time < time_stamp) {
    return true;
  }
  if (FLAGS_timeout < cur_time - time_stamp) {
    return true;
  }
  return false;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::MsgHandle(int fd) {
  
    struct sockaddr_in client_addr;
    socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
    int total_recv = 2 * FRAME_SIZE;
    int bytes = 0;
    char total_buf[2 * FRAME_SIZE] = {0};

    while(true){

    bytes =
        static_cast<int>(recvfrom(fd, total_buf, total_recv, 0,
                                  (struct sockaddr *)&client_addr, &sock_len));
    // std::cout << "total recv " << total_buf << " "<<bytes << " fd: " << fd << std::endl;
    if (bytes <= 0 || bytes > total_recv) {
      // return false;
      continue;
    }
    char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
    size_t offset = 0;
    memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
    if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
      std::cout << "header flag not match!" << std::endl;;
      // return false;
      continue;
    }
    offset += sizeof(BRIDGE_HEADER_FLAG) + 1;

    char header_size_buf[sizeof(hsize) + 1] = {0};
    const char *cursor = total_buf + offset;
    memcpy(header_size_buf, cursor, sizeof(hsize));
    hsize header_size = *(reinterpret_cast<hsize *>(header_size_buf));
    if (header_size > FRAME_SIZE) {
      std::cout << "header size is more than FRAME_SIZE!" << std::endl;;
      continue;
      // return false;
    }
    offset += sizeof(hsize) + 1;

    BridgeHeader header;
    size_t buf_size = header_size - offset;
    cursor = total_buf + offset;
    if (!header.Diserialize(cursor, buf_size)) {
      std::cout << "header diserialize failed!" << std::endl;;
      continue;
      // return false;
    }
    // std::cout << "proto name : " << header.GetMsgName().c_str() << std::endl;
    // std::cout << "proto sequence num: " << header.GetMsgID() << std::endl;
    // std::cout << "proto total frames: " << header.GetTotalFrames() << std::endl;
    // std::cout << "proto frame index: " << header.GetIndex() << std::endl;

    // std::cout << header.GetMsgName().c_str() << std::endl;
    // std::cout << "MsgID : "<< header.GetMsgID() << std::endl;
    // std::cout << header.GetMsgSize() << std::endl;
    // std::cout << header.GetFrameSize() << std::endl;
    // // std::cout <<" Framecnt : "<< frame_cnt << std::endl;
    
    // std::cout << "Msg Name   : " << header.GetMsgName().c_str() << std::endl;
    // std::cout << "Msg Name   : " << header.GetMsgName() << std::endl;
    // std::cout << "proto Name   : " << proto_name_ << std::endl;
    // std::cout << "topic Name   : " << topic_name_ << std::endl;
    // std::cout << "GetIndex   : " << header.GetIndex() << std::endl;
    // std::cout << "TotalFrame : " << header.GetTotalFrames() << std::endl;
    // std::cout << "=========================================== "<< std::endl;

    std::lock_guard<std::mutex> lock(mutex_);

    if(header.GetIndex() == 0){
      proto_buf_ = CreateBridgeProtoBuf(header);
      proto_buf_->Initialize(header);
    }

    if (!proto_buf_) {
      continue;
      // return false;
    }

    cursor = total_buf + header_size;
    char *buf = proto_buf_->GetBuf(header.GetFramePos());
    memcpy(buf, cursor, header.GetFrameSize());
    proto_buf_->UpdateStatus(header.GetIndex());
    // std::cout << "index : " << header.GetIndex() << std::endl; 
    // std::cout << "IsreadyDiser : " << proto_buf_->IsReadyDiserialize() << std::endl; 
    if (proto_buf_->IsReadyDiserialize()) {
      
      auto pb_msg = std::make_shared<T>();
      proto_buf_->Diserialized(pb_msg);
      {
        std::lock_guard<std::mutex> lock(apollo_ctrl_cmd_mutex_);
        proto_msg_buf_.push(pb_msg);
      }
      // if(proto_name_ == "CorrectedImu"){
      // if(proto_name_ == "Chassis"){
      //   std::cout << pb_msg->throttle_percentage() << std::endl;
      // }
    //   writer_->Write(pb_msg);
      RemoveInvalidBuf(proto_buf_->GetMsgID());
      RemoveItem(&proto_list_, proto_buf_);
      proto_init_flag_ = true;

    }else{
      if(header.GetIndex())
      proto_init_flag_ = false;
    }
    


    }

  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::RemoveInvalidBuf(uint32_t msg_id) {
  if (msg_id == 0) {
    return false;
  }
  typename std::vector<BridgeProtoDiserializedBuf<T> *>::iterator itor =
      proto_list_.begin();
  for (; itor != proto_list_.end();) {
    if ((*itor)->GetMsgID() < msg_id) {
      BridgeProtoDiserializedBuf<T> *tmp = *itor;
      FREE_POINTER(tmp);
      itor = proto_list_.erase(itor);
      continue;
    }
    ++itor;
  }
  return true;
}

}  // namespace bridge
}  // namespace apollo
