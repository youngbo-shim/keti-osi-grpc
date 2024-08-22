#include "keti_ros_bridge.h"
#define STEER_RATIO 12.3

KetiROSBridge::KetiROSBridge(std::string client_ip_address, std::string server_ip_address) : OSIBridge(client_ip_address, server_ip_address)
{
  std::string hdmap_path, hdmap_from, lane_filename, link_filename, node_filename, speedbump_filename,
              safetysign_filename, surfacemark_filename, parkinglot_filename, drivewaysection_filename,
              trafficlight_filename, postpoint_filename, vehicleprotectionsafety_filename, heightbarrier_filename,
              intersection_filename;
  
  nh_.getParam("data_path", hdmap_path);
  nh_.getParam("hdmap_from", hdmap_from);
  nh_.getParam("lane_filename",lane_filename);
  nh_.getParam("link_filename",link_filename);
  nh_.getParam("node_filename",node_filename);
  nh_.getParam("speedbump_filename",speedbump_filename);            
  nh_.getParam("safetysign_filename",safetysign_filename); 
  nh_.getParam("surfacemark_filename",surfacemark_filename);
  nh_.getParam("parkinglot_filename",parkinglot_filename);
  nh_.getParam("drivewaysection_filename",drivewaysection_filename);
  nh_.getParam("trafficlight_filename",trafficlight_filename);
  nh_.getParam("postpoint_filename",postpoint_filename);
  nh_.getParam("vehicleprotectionsafety_filename",vehicleprotectionsafety_filename);
  nh_.getParam("heightbarrier_filename",heightbarrier_filename);
  nh_.getParam("intersection_filename",intersection_filename);

  hdmap_.reset(new HDMap(hdmap_from, hdmap_path, lane_filename,
                  link_filename, node_filename, speedbump_filename,
                  safetysign_filename, surfacemark_filename, parkinglot_filename,
                  drivewaysection_filename, trafficlight_filename, postpoint_filename,
                  vehicleprotectionsafety_filename, heightbarrier_filename, intersection_filename));

  sub_is_changed_offset_ = nh_.subscribe("/is_changed_offset",1, &KetiROSBridge::CallbackUpdateOffsetParams, this);
  sub_keti_cmd_ = nh_.subscribe("/vehicle_cmd", 1, &KetiROSBridge::CallbackKetiCmd, this);

  XmlRpc::XmlRpcValue camera_param, lidar_param, radar_param;
  nh_.getParam("camera_param", camera_param);
  nh_.getParam("lidar_param", lidar_param);
  nh_.getParam("radar_param", radar_param);

  std::cout << "get param" << std::endl;

  num_of_camera_ = camera_param.size();
  num_of_lidar_ = lidar_param.size();
  num_of_radar_ = radar_param.size();

  pub_imgs_.resize(num_of_camera_);
  pub_clouds_.resize(num_of_lidar_);
  pub_radar_.resize(num_of_radar_);

  for (size_t i = 0; i < camera_param.size(); i++){
    std::string topic_name = "camera" + std::to_string(i) + "/grpc/compressed";
    pub_imgs_[i] = nh_.advertise<sensor_msgs::CompressedImage>(topic_name, 1);
  }

  for (size_t i = 0; i < lidar_param.size(); i++){
    std::string topic_name = "lidar" + std::to_string(i) + "/grpc/points";
    pub_clouds_[i] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  }

  for (size_t i = 0; i < radar_param.size(); i++){
    std::string topic_name = "radar" + std::to_string(i) + "/grpc/radar";
    pub_radar_[i] = nh_.advertise<radar_msgs::RadarScan>(topic_name, 1);
  }

  pub_tls_ = nh_.advertise<perception_msgs::TrafficLights>("/traffic_lights", 1);
  pub_objs_ = nh_.advertise<cyber_perception_msgs::PerceptionObstacles>("/obstacles_local", 1);
  pub_vehicle_state_ = nh_.advertise<control_msgs::VehicleState>("/vehicle_state", 1);
  pub_current_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);

  std::cout << "finished" << std::endl;

  is_initialized_ = true;
}

void KetiROSBridge::StartBridge(){
  if(!is_initialized_){
    std::cout << "KETI ROS Bridge has Not been initiated!" << std::endl;
    return;
  }

  converting_thread_ = std::thread(&KetiROSBridge::ConvertThread, this);  
  publish_thread_ = std::thread(&KetiROSBridge::PublishThread, this);
  ros::spin();
}

void KetiROSBridge::Stop(){
  converting_thread_.join();
  publish_thread_.join();
}

void KetiROSBridge::CallbackUpdateOffsetParams(const std_msgs::Bool& check){

  std::cout << "hdmap reset start offset" << std::endl;
  std::string hdmap_path, hdmap_from, lane_filename, link_filename, node_filename, speedbump_filename,
              safetysign_filename, surfacemark_filename, parkinglot_filename, drivewaysection_filename,
              trafficlight_filename, postpoint_filename, vehicleprotectionsafety_filename, heightbarrier_filename,
              intersection_filename;

  nh_.getParam("data_path", hdmap_path);
  nh_.getParam("hdmap_from", hdmap_from);
  nh_.getParam("lane_filename",lane_filename);
  nh_.getParam("link_filename",link_filename);
  nh_.getParam("node_filename",node_filename);
  nh_.getParam("speedbump_filename",speedbump_filename);
  nh_.getParam("safetysign_filename",safetysign_filename); 
  nh_.getParam("surfacemark_filename",surfacemark_filename);
  nh_.getParam("parkinglot_filename",parkinglot_filename);
  nh_.getParam("drivewaysection_filename",drivewaysection_filename);
  nh_.getParam("trafficlight_filename",trafficlight_filename);
  nh_.getParam("postpoint_filename",postpoint_filename);
  nh_.getParam("vehicleprotectionsafety_filename",vehicleprotectionsafety_filename);
  nh_.getParam("heightbarrier_filename",heightbarrier_filename);
  nh_.getParam("intersection_filename",intersection_filename);

  hdmap_.reset(new HDMap(hdmap_from, hdmap_path, lane_filename,
                  link_filename, node_filename, speedbump_filename,
                  safetysign_filename, surfacemark_filename, parkinglot_filename,
                  drivewaysection_filename, trafficlight_filename, postpoint_filename,
                  vehicleprotectionsafety_filename, heightbarrier_filename, intersection_filename));

  std::cout << "reset hdmap!" << std::endl;
}

void KetiROSBridge::CallbackKetiCmd(const control_msgs::VehicleCMD& keti_cmd) {
  SensorView cmd_sensor_view;
  
  constexpr double a_grav = 9.8;
  cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_steering()
                ->mutable_vehicle_steering_wheel()->set_angle(math::deg2rad(keti_cmd.steer_angle_cmd/STEER_RATIO));
  if ( keti_cmd.AEB_decel_cmd > (std::abs(keti_cmd.accel_decel_cmd)/a_grav) ){
    cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_motion()->mutable_acceleration()->set_x(- a_grav * keti_cmd.AEB_decel_cmd);
  }
  else{
    cmd_sensor_view.mutable_host_vehicle_data()->mutable_vehicle_motion()->mutable_acceleration()->set_x(keti_cmd.accel_decel_cmd);
  }
  server_sensor_view_buf_.push(cmd_sensor_view);
}

void KetiROSBridge::ConvertThread(){ 
  std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table, radar_seq_table;
  size_t tl_seq = 0, obj_seq = 0, ego_vehicle_state_seq = 0;
  
  for(int i = 0 ; i < num_of_lidar_ ; i++){
    lidar_seq_table[i] = 0;  
  }

  for(int i = 0 ; i < num_of_camera_ ; i++){
    camera_seq_table[i] = 0;  
  }

  for(int i = 0 ; i < num_of_radar_ ; i++){
    radar_seq_table[i] = 0;  
  }    

  while(1){
    SensorView sensor_view;
    if(!sensor_view_buf_.empty()){
        std::lock_guard<std::mutex> lock(sensor_view_mutex_);
        sensor_view.CopyFrom(sensor_view_buf_.front());
        sensor_view_buf_.pop();
    } else {
      continue;
    }

    for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
      auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
      int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();
      size_t seq = lidar_seq_table[lidar_id];

      //TODO(All): FMU-LiDAR 센서 에러 모델링
      fmi2CallbackFunctions callbacks = {
        .logger = NULL,
        .allocateMemory = NULL,
        .freeMemory = NULL,
        .stepFinished = NULL,
        .componentEnvironment = NULL
      };

      COSMPDummySensor osmp_lidar("LiDAR", fmi2CoSimulation, "1", "", &callbacks, false, false);    
      
      int size = sensor_view.ByteSizeLong();
      void* buffer = malloc(size);
      sensor_view.SerializeToArray(buffer, size);  

      std::cout << "bytes : " << sensor_view.ByteSizeLong() << std::endl;

      size_t num_var = 3;
      fmi2ValueReference vr_in[num_var] = { FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX,
                                            FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX,
                                            FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX };

      int address_hi = 0, address_lo = 0;
      osmp_lidar.encode_pointer_to_integer(buffer, address_hi, address_lo);
      fmi2Integer value[num_var] = { address_lo, address_hi, size };

      std::string* tmp = (std::string*)osmp_lidar.decode_integer_to_pointer(address_hi, address_lo);

      std::cout << "address_hi, address_lo : " << buffer << ", " << tmp << ", " << address_hi << ", " << address_lo << std::endl;

      osmp_lidar.SetInteger(vr_in, num_var, value);
      
      // 현재 시간 가져오기
      auto now = std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      double seconds = std::chrono::duration<double>(duration).count();

      fmi2Real currentCommunicationPoint = seconds;
      fmi2Real communicationStepSize = 0.1;
      fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component = false;

      osmp_lidar.DoStep( currentCommunicationPoint,
                        communicationStepSize,
                        noSetFMUStatePriorToCurrentPointfmi2Component);

      fmi2ValueReference vr_out[num_var] = { FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX,
                                            FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX,
                                            FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX };

      fmi2Integer out_value[num_var];
      osmp_lidar.GetInteger(vr_out, num_var, out_value);

      auto out_buffer = osmp_lidar.decode_integer_to_pointer(out_value[1], out_value[0]);

      osi3::SensorData sensor_data;
      sensor_data.ParseFromArray(out_buffer, out_value[2]);

      lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_data.sensor_view(0).lidar_sensor_view(0));
      ///////////////////////////////////////////////////////////////////////////////////

      msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&KetiROSConverter::ProcLidar, &converter_, lidar_sensor_view, lidar_id, seq));
      lidar_seq_table[lidar_id]++;
    }

    for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
      auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
      int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
      size_t seq = camera_seq_table[camera_id];

      msg_result_.camera_res_table[camera_id].push(keti::task::Async(&KetiROSConverter::ProcCamera, &converter_, camera_sensor_view, camera_id, seq));
      camera_seq_table[camera_id]++;
    }

    for ( int i = 0 ; i < sensor_view.radar_sensor_view().size() ; i++ ) {
      auto radar_sensor_view = std::make_shared<RadarSensorView>(sensor_view.radar_sensor_view(i));
      int radar_id = radar_sensor_view->view_configuration().sensor_id().value();
      size_t seq = radar_seq_table[radar_id];

      msg_result_.radar_res_table[radar_id].push(keti::task::Async(&KetiROSConverter::ProcRadar, &converter_, radar_sensor_view, radar_id, seq));
      radar_seq_table[radar_id]++;
    }

    if ( sensor_view.global_ground_truth().traffic_light_size() > 0 ){
      auto osi_tls = std::make_shared<RepeatedPtrField<osi3::TrafficLight>>(sensor_view.global_ground_truth().traffic_light());

      msg_result_.tl_res_table.push(keti::task::Async(&KetiROSConverter::ProcTL, &converter_, osi_tls, hdmap_, tl_seq));
      tl_seq++;
    }

    if ( sensor_view.global_ground_truth().moving_object_size() > 0 || sensor_view.global_ground_truth().stationary_object_size() > 0 ){
      auto moving_objs = std::make_shared<RepeatedPtrField<MovingObject>>(sensor_view.global_ground_truth().moving_object());
      auto stationary_objs = std::make_shared<RepeatedPtrField<StationaryObject>>(sensor_view.global_ground_truth().stationary_object());

      msg_result_.obj_res_table.push(keti::task::Async(&KetiROSConverter::ProcObj, &converter_, moving_objs, stationary_objs, obj_seq));
      obj_seq++;
    }

    if(sensor_view.host_vehicle_data().vehicle_motion().has_acceleration()){
      auto ego_vehicle_state_view = std::make_shared<HostVehicleData>(sensor_view.host_vehicle_data());
      msg_result_.ego_state_res_table.push(keti::task::Async(&KetiROSConverter::ProcEgoVehicleState, &converter_, 
                                                              ego_vehicle_state_view, ego_vehicle_state_seq, hdmap_));
      ego_vehicle_state_seq++;
    }
  }
}

void KetiROSBridge::PublishThread(){
  while(true){
    for(int i = 0 ; i < num_of_lidar_ ; i++){
      if(msg_result_.lidar_res_table.count(i) == 0){
        continue;
      }

      if(msg_result_.lidar_res_table[i].empty()){
        continue;
      }

      try {
        auto msg = msg_result_.lidar_res_table[i].front().get();        
        pub_clouds_[i].publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Lidar)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Lidar): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Lidar): " << e.what() << std::endl;
      }
      msg_result_.lidar_res_table[i].pop();
    }

    for(int i = 0 ; i < num_of_camera_ ; i++){
      if(msg_result_.camera_res_table.count(i) == 0){
        continue;
      }

      if(msg_result_.camera_res_table[i].empty()){
        continue;
      }

      try {
        auto msg = msg_result_.camera_res_table[i].front().get();        
        pub_imgs_[i].publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Image)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Image): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Image): " << e.what() << std::endl;
      }
      msg_result_.camera_res_table[i].pop();
    }

    for(int i = 0 ; i < num_of_radar_ ; i++){
      if(msg_result_.radar_res_table.count(i) == 0){
        continue;
      }

      if(msg_result_.radar_res_table[i].empty()){
        continue;
      }

      try{
        auto msg = msg_result_.radar_res_table[i].front().get();
        pub_radar_[i].publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Radar)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Radar): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Radar): " << e.what() << std::endl;
      }
      msg_result_.radar_res_table[i].pop();
    }

    if(!msg_result_.tl_res_table.empty()){
      try {
        auto msg = msg_result_.tl_res_table.front().get();
        pub_tls_.publish(msg.second);        
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Traffic)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Traffic): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Traffic): " << e.what() << std::endl;
      }
      msg_result_.tl_res_table.pop();
    }

    if(!msg_result_.obj_res_table.empty()){
      try {
        auto msg = msg_result_.obj_res_table.front().get();
        pub_objs_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Obstacle)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Obstacle): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Obstacle): " << e.what() << std::endl;
      }
      msg_result_.obj_res_table.pop();
    }

    if(!msg_result_.ego_state_res_table.empty()){
      try {
        auto msg = msg_result_.ego_state_res_table.front().get();
        pub_vehicle_state_.publish(std::get<1>(msg));
        pub_current_pose_.publish(std::get<2>(msg));

        static tf::TransformBroadcaster odom_broadcaster;
        odom_broadcaster.sendTransform(std::get<3>(msg));
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(VehicleState)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(VehicleState): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(VehicleState): " << e.what() << std::endl;
      }
      msg_result_.ego_state_res_table.pop();
    }
  }
}