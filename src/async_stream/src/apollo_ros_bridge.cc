#include "apollo_ros_bridge.h"
#define STEER_RATIO 12.3
// ApolloROSBridge::ApolloROSBridge(std::string ip_address)
//           : OSIBridge(ip_address)
ApolloROSBridge::ApolloROSBridge(std::string client_ip_address, std::string server_ip_address)
          : OSIBridge(client_ip_address, server_ip_address)          
{
  XmlRpc::XmlRpcValue camera_param, lidar_param;
  nh_.getParam("camera_param", camera_param);
  nh_.getParam("lidar_param", lidar_param);
  num_of_camera_ = camera_param.size();
  num_of_lidar_ = lidar_param.size();

  is_apollo_cmd_sub_initialized_ = sub_ctrl_cmd_.Init();
  if(!is_apollo_cmd_sub_initialized_)
    std::cout << "apollo_cmd_sub is not initialized" << std::endl;
  is_initialized_ = true;

}

void ApolloROSBridge::StartBridge(){
  if(!is_initialized_){
    std::cout << "ApolloROSBridge Converter has Not been initiated!" << std::endl;
    return;
  }
  std::cout << "ApolloROSBridge::StartBridge " << std::endl;
  converting_thread_ = std::thread(&ApolloROSBridge::ConvertThread, this);
  publish_thread_ = std::thread(&ApolloROSBridge::PublishThread, this);
  ros::spin();
}

void ApolloROSBridge::Stop(){
  converting_thread_.join();
  publish_thread_.join();
}

void ApolloROSBridge::ConvertThread(){ 
  std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table;
  size_t tl_seq = 0, obj_seq = 0, ego_vehicle_state_seq = 0;
  for(int i = 0 ; i < num_of_lidar_ ; i++){
    lidar_seq_table[i] = 0;  
  }

  for(int i = 0 ; i < num_of_camera_ ; i++){
    camera_seq_table[i] = 0;  
  }

  while(1){

    SensorView sensor_view;
    apollo::control::ControlCommand apollo_ctrl_cmd;
    if(!sensor_view_buf_.empty()){
      std::lock_guard<std::mutex> lock(sensor_view_mutex_);
      sensor_view.CopyFrom(sensor_view_buf_.front());
      sensor_view_buf_.pop();
      is_sensor_view_buf_empty_ = false;
    }
    else
      is_sensor_view_buf_empty_= true;
    if(!sub_ctrl_cmd_.GetProtoMsgBuf().empty()){
      // std::cout << "sub_ctrl_cmd_.GetProtoMsgBuf() size: " << sub_ctrl_cmd_.GetProtoMsgBuf().size() << std::endl;
      std::lock_guard<std::mutex> lock(sub_ctrl_cmd_.getApolloCtrlCmdMutex());
      apollo_ctrl_cmd.CopyFrom(*sub_ctrl_cmd_.GetProtoMsgBuf().front().get());
      sub_ctrl_cmd_.GetProtoMsgBuf().pop();
      is_apollo_cmd_ctrl_buf_empty_ = false;
      // std::cout << "(after pop) sub_ctrl_cmd_.GetProtoMsgBuf() size: " << sub_ctrl_cmd_.GetProtoMsgBuf().size() << std::endl;
    }
    else
      is_apollo_cmd_ctrl_buf_empty_ = true;

    if(is_sensor_view_buf_empty_ && is_apollo_cmd_ctrl_buf_empty_)
      continue;

    if(!is_apollo_cmd_ctrl_buf_empty_){
      SensorView cmd_sensor_view;
      cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_steering()
                    ->mutable_vehicle_steering_wheel()->set_angle(math::deg2rad((apollo_ctrl_cmd.steering_target() * 3.5) / STEER_RATIO));
      cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_powertrain()->set_pedal_position_acceleration( (apollo_ctrl_cmd.throttle() * 0.8 )/ 100.0);
      cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_brake_system()->set_pedal_position_brake(apollo_ctrl_cmd.brake() / 100.0);
      server_sensor_view_buf_.push(cmd_sensor_view);
    }
    
    if(!is_sensor_view_buf_empty_){
      for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
        auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
        int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
        size_t seq = camera_seq_table[camera_id];
        msg_result_.camera_res_table[camera_id].push(keti::task::Async(&ApolloROSConverter::ProcCamera, 
                                                                      &converter_,
                                                                      camera_sensor_view,
                                                                      camera_id, seq));
        camera_seq_table[camera_id]++;
      }

      // for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
      //   auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
      //   int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();
      //   size_t seq = lidar_seq_table[lidar_id];
      //   msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&ApolloROSConverter::ProcLidar,
      //                                                                 &converter_,
      //                                                                 lidar_sensor_view,
      //                                                                 lidar_id, seq));
      //   lidar_seq_table[lidar_id]++;
      // }

      if(sensor_view.host_vehicle_data().has_vehicle_motion()){
        if(sensor_view.host_vehicle_data().vehicle_motion().has_acceleration()){

          msg_result_.gnss_best_pose_res_table.push(keti::task::Async(&ApolloROSConverter::ProcBestPose,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));

          msg_result_.imu_res_table.push(keti::task::Async(&ApolloROSConverter::ProcImu,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));

          msg_result_.corimu_res_table.push(keti::task::Async(&ApolloROSConverter::ProcCorImu,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));

          msg_result_.odom_res_table.push(keti::task::Async(&ApolloROSConverter::ProcOdometry,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));

          msg_result_.chassis_res_table.push(keti::task::Async(&ApolloROSConverter::ProcVehicleChassis,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));

          // Needs to be moved to other place 
          msg_result_.traffic_light_res_table.push(keti::task::Async(&ApolloROSConverter::ProcTrafficLightDetection,
                                                          &converter_,
                                                          sensor_view.host_vehicle_data(),
                                                          ego_vehicle_state_seq));
          
          msg_result_.ins_res_table.push(keti::task::Async(&ApolloROSConverter::ProcIns,
                                                          &converter_,
                                                          ego_vehicle_state_seq));

          ego_vehicle_state_seq++;
        }
      }

      if ( sensor_view.global_ground_truth().moving_object_size() > 0 || sensor_view.global_ground_truth().stationary_object_size() > 0 ){
        auto moving_objs = std::make_shared<RepeatedPtrField<MovingObject>>(sensor_view.global_ground_truth().moving_object());
        auto stationary_objs = std::make_shared<RepeatedPtrField<StationaryObject>>(sensor_view.global_ground_truth().stationary_object());

        msg_result_.perception_obs_res_table.push(keti::task::Async(&ApolloROSConverter::ProcPerceptionObstacles, &converter_, moving_objs, stationary_objs, obj_seq));
        obj_seq++;
      }

    }
  }
}

template <typename T>
bool ApolloROSBridge::SendData(const T &proto_msg,
                const std::string proto_msg_name,
                const std::string remote_ip,
                const uint16_t remote_port){

  struct sockaddr_in server_addr;
  server_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(remote_port);

  // std::cout << "connecting to server2... ";

  int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

  int res =
      connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (res < 0) {
    std::cout << "connected server failed " << std::endl;
  }

  // std::cout << "connected to server2 success. port [" << remote_port << "]" << std::endl;

  // BridgeProtoSerializedBuf<GnssBestPose> proto_buf;
  BridgeProtoSerializedBuf<T> proto_buf;
  proto_buf.Serialize(proto_msg, proto_msg_name);
  for (size_t j = 0; j < proto_buf.GetSerializedBufCount(); j++) {
    ssize_t nbytes = send(sock_fd, proto_buf.GetSerializedBuf(j),
                          proto_buf.GetSerializedBufSize(j), 0);
    if (nbytes != static_cast<ssize_t>(proto_buf.GetSerializedBufSize(j))) {
      std::cout <<"proto_msg_name: " << proto_msg_name << std::endl;
      std::cout << "sent msg failed " << std::endl;
      break;
    }
    // std::cout  << "sent " << nbytes << " bytes to server with sequence num " << std::endl;
  }
  close(sock_fd);

  // 1000Hz
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return true;
}

void ApolloROSBridge::PublishThread(){
  while(true){    
    // for(int i = 0 ; i < num_of_lidar_ ; i++){
    //   if(msg_result_.lidar_res_table.count(i) == 0){
    //     continue;
    //   }

    //   if(msg_result_.lidar_res_table[i].empty()){
    //     continue;
    //   }

    //   try {
    //     auto msg = msg_result_.lidar_res_table[i].front().get();      
    //     pub_clouds_[i].publish(msg.second);
    //   }catch (const std::future_error& e) {
    //       if (e.code() == std::make_error_code(std::future_errc::no_state)) {
    //           std::cerr << "No associated state(Lidar)" << std::endl;
    //       } else {
    //           std::cerr << "Caught std::future_error(Lidar): " << e.what() << std::endl;
    //       }
    //   } catch (const std::exception& e) {
    //       std::cerr << "Caught exception(Lidar): " << e.what() << std::endl;
    //   }
    //   msg_result_.lidar_res_table[i].pop();
    // }
    
    for(int i = 0 ; i < num_of_camera_ ; i++){
      if(msg_result_.camera_res_table.count(i) == 0){
        continue;
      }

      if(msg_result_.camera_res_table[i].empty()){
        continue;
      }
      auto pb_msg = msg_result_.camera_res_table[i].front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15019;
      SendData(pb_msg.second, "CompressedImage", remote_ip, remote_port);
      msg_result_.camera_res_table[i].pop();
      // std::cout << "after pop camera_res_table size: " << msg_result_.camera_res_table.size() << std::endl;

    }

    if(!msg_result_.gnss_best_pose_res_table.empty()){

      auto pb_msg = msg_result_.gnss_best_pose_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15011;
      SendData(pb_msg.second, "BestPose", remote_ip, remote_port);

      msg_result_.gnss_best_pose_res_table.pop();
      // std::cout << "after pop gnss_best_pose_res_table size: " << msg_result_.gnss_best_pose_res_table.size() << std::endl;

    }

    if(!msg_result_.imu_res_table.empty()){

      auto pb_msg = msg_result_.imu_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15014;
      SendData(pb_msg.second, "Imu", remote_ip, remote_port);

      msg_result_.imu_res_table.pop();
      // std::cout << "after pop imu_res_table size: " << msg_result_.imu_res_table.size() << std::endl;

    }

    if(!msg_result_.corimu_res_table.empty()){

      auto pb_msg = msg_result_.corimu_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15013;
      SendData(pb_msg.second, "CorrectedImu", remote_ip, remote_port);

      msg_result_.corimu_res_table.pop();
      // std::cout << "after pop corimu_res_table size: " << msg_result_.corimu_res_table.size() << std::endl;
    }

    if(!msg_result_.odom_res_table.empty()){

      auto pb_msg = msg_result_.odom_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15016;
      SendData(pb_msg.second, "Odometry", remote_ip, remote_port);

      msg_result_.odom_res_table.pop();
      // std::cout << "after pop odom_res_table size: " << msg_result_.odom_res_table.size() << std::endl;

    }

    if(!msg_result_.chassis_res_table.empty()){

      auto pb_msg = msg_result_.chassis_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15012;
      SendData(pb_msg.second, "Chassis", remote_ip, remote_port);

      msg_result_.chassis_res_table.pop();
      // std::cout << "after pop chassis_res_table size: " << msg_result_.chassis_res_table.size() << std::endl;
    }

    if(!msg_result_.traffic_light_res_table.empty()){

      auto pb_msg = msg_result_.traffic_light_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 7502;
      SendData(pb_msg.second, "TrafficLight", remote_ip, remote_port);

      msg_result_.traffic_light_res_table.pop();
      // std::cout << "after pop traffic_light_res_table size: " << msg_result_.traffic_light_res_table.size() << std::endl;
    }

    if(!msg_result_.perception_obs_res_table.empty()){

      auto pb_msg = msg_result_.perception_obs_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 7505;
      SendData(pb_msg.second, "PerceptionObstacles", remote_ip, remote_port);

      msg_result_.perception_obs_res_table.pop();
      // std::cout << "after pop perception_obs_res_table size: " << msg_result_.perception_obs_res_table.size() << std::endl;
    }

    if(!msg_result_.ins_res_table.empty()){

      auto pb_msg = msg_result_.ins_res_table.front().get();
      std::string remote_ip = "127.0.0.1";
      uint16_t remote_port = 15015;
      SendData(pb_msg.second, "InsStat", remote_ip, remote_port);

      msg_result_.ins_res_table.pop();
      // std::cout << "after pop ins_res_table size: " << msg_result_.ins_res_table.size() << std::endl;

    }


  }
}
