#include "autoware_ros_bridge.h"

AutowareROSBridge::AutowareROSBridge(std::string ip_address)
          : OSIBridge(ip_address)
{
  XmlRpc::XmlRpcValue camera_param, lidar_param;

  nh_.getParam("camera_param", camera_param);
  nh_.getParam("lidar_param", lidar_param);

  std::cout << "get param" << std::endl;

  num_of_camera_ = camera_param.size();
  num_of_lidar_ = lidar_param.size();

  pub_imgs_.resize(num_of_camera_);
  pub_clouds_.resize(num_of_lidar_);

  for (size_t i = 0; i < camera_param.size(); i++){
    std::string topic_name = "camera" + std::to_string(i) + "/grpc/compressed";
    pub_imgs_[i] = nh_.advertise<sensor_msgs::CompressedImage>(topic_name, 1);
  }

  for (size_t i = 0; i < lidar_param.size(); i++){
    std::string topic_name = "lidar" + std::to_string(i) + "/grpc/points";
    pub_clouds_[i] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  }

  pub_objs_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/grpc/objects", 1);
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/grpc/imu", 1);
  pub_gps_ = nh_.advertise<morai_msgs::GPSMessage>("/grpc/gps", 1);
  pub_autoware_ego_state_ = nh_.advertise<geometry_msgs::PoseStamped>("/grpc/autoware_ego_vehicle_state", 1);
  pub_autoware_ego_speed_ = nh_.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1);

  nh_.getParam("x_offset", x_offset_);
  nh_.getParam("y_offset", y_offset_);

  converter_.SetoffsetX(x_offset_);
  converter_.SetoffsetY(y_offset_);

  is_initialized_ = true;

}

void AutowareROSBridge::StartBridge(){
  if(!is_initialized_){
    std::cout << "AutowareROSBridge Converter has Not been initiated!" << std::endl;
    return;
  }

  converting_thread_ = std::thread(&AutowareROSBridge::ConvertThread, this);
  publish_thread_ = std::thread(&AutowareROSBridge::PublishThread, this);
  ros::spin();
}

void AutowareROSBridge::Stop(){
  converting_thread_.join();
  publish_thread_.join();
}

void AutowareROSBridge::ConvertThread(){ 
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
    if(sensor_view_buf_.size() != 0){
        std::lock_guard<std::mutex> lock(sensor_view_mutex_);
        sensor_view = sensor_view_buf_.front();
        sensor_view_buf_.pop();
    } else continue;

    for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
      auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
      int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();
      size_t seq = lidar_seq_table[lidar_id];
      msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&AutowareROSConverter::ProcLidar, &converter_, lidar_sensor_view, lidar_id, seq));
      lidar_seq_table[lidar_id]++;
    }

    for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
      auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
      int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
      size_t seq = camera_seq_table[camera_id];
      msg_result_.camera_res_table[camera_id].push(keti::task::Async(&AutowareROSConverter::ProcCamera, &converter_, camera_sensor_view, camera_id, seq));
      camera_seq_table[camera_id]++;
    }

    if ( sensor_view.global_ground_truth().moving_object_size() > 0 || sensor_view.global_ground_truth().stationary_object_size() > 0 ){
      auto moving_objs = std::make_shared<RepeatedPtrField<MovingObject>>(sensor_view.global_ground_truth().moving_object());
      auto stationary_objs = std::make_shared<RepeatedPtrField<StationaryObject>>(sensor_view.global_ground_truth().stationary_object());
      msg_result_.obj_res_table.push(keti::task::Async(&AutowareROSConverter::ProcObj, &converter_, moving_objs, stationary_objs, obj_seq));
      obj_seq++;
    }

    if(sensor_view.host_vehicle_data().vehicle_motion().has_acceleration()){
      auto imu_sensor_view = std::make_shared<HostVehicleData>(sensor_view.host_vehicle_data());
      auto gps_sensor_view = std::make_shared<HostVehicleData>(sensor_view.host_vehicle_data());
      auto ego_vehicle_state_view = std::make_shared<HostVehicleData>(sensor_view.host_vehicle_data());
      msg_result_.imu_res_table.push(keti::task::Async(&AutowareROSConverter::ProcImu, &converter_, imu_sensor_view, ego_vehicle_state_seq));
      msg_result_.gps_res_table.push(keti::task::Async(&AutowareROSConverter::ProcGps, &converter_, gps_sensor_view, ego_vehicle_state_seq));
      msg_result_.ego_state_res_table.push(keti::task::Async(&AutowareROSConverter::ProcEgoVehicleState, &converter_, ego_vehicle_state_view, ego_vehicle_state_seq));
      msg_result_.ego_speed_res_table.push(keti::task::Async(&AutowareROSConverter::ProcEgoVehicleSpeed, &converter_, ego_vehicle_state_view, ego_vehicle_state_seq));
      ego_vehicle_state_seq++;
    }
  }
}

void AutowareROSBridge::PublishThread(){

  int thread_start_cnt = 0;

  while(true){
    if(thread_start_cnt < 100){
      std::cout << "start publish thread" << std::endl;
      thread_start_cnt++;
    }

    for(int i = 0 ; i < num_of_lidar_ ; i++){
      if(msg_result_.lidar_res_table.count(i) == 0){
        continue;
      }

      if(msg_result_.lidar_res_table[i].size() == 0){
        continue;
      }

      try {
        auto msg = msg_result_.lidar_res_table[i].front().get();      
        pub_clouds_[i].publish(msg.second);
      }catch (const std::future_error& e) {
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

      if(msg_result_.camera_res_table[i].size() == 0){
        continue;
      }
      try {
        auto msg = msg_result_.camera_res_table[i].front().get();
        pub_imgs_[i].publish(msg.second);
      }catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Camera)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Camera): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Camera): " << e.what() << std::endl;
      }
      msg_result_.camera_res_table[i].pop();
    }

    if(msg_result_.obj_res_table.size() != 0){
      try {
        auto msg = msg_result_.obj_res_table.front().get();
        pub_objs_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Camera)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Camera): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Camera): " << e.what() << std::endl;
      }
      msg_result_.obj_res_table.pop();
    }

    if(msg_result_.imu_res_table.size() != 0){
      try {
        auto msg = msg_result_.imu_res_table.front().get();
        pub_imu_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(Imu)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(Imu): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(Imu): " << e.what() << std::endl;
      }
      msg_result_.imu_res_table.pop();
    }

    if(msg_result_.gps_res_table.size() != 0){
      try {
        auto msg = msg_result_.gps_res_table.front().get();
        pub_gps_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(gps)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(gps): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(gps): " << e.what() << std::endl;
      }
      msg_result_.gps_res_table.pop();
    }

    if(msg_result_.ego_state_res_table.size() != 0){
      try {
        auto msg = msg_result_.ego_state_res_table.front().get();
        { 
          // publish map to base_link tf
          static tf::TransformBroadcaster br;
          br.sendTransform(
            tf::StampedTransform(
              tf::Transform(tf::Quaternion(msg.second.pose.orientation.x, msg.second.pose.orientation.y, msg.second.pose.orientation.z, msg.second.pose.orientation.w),
              tf::Vector3(msg.second.pose.position.x, msg.second.pose.position.y, 0.0)),
              ros::Time::now(),
              "map",
              "base_link"
          ));
        }
        pub_autoware_ego_state_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(ego state)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(ego state): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(ego state): " << e.what() << std::endl;
      }
      msg_result_.ego_state_res_table.pop();
    }         

    if(msg_result_.ego_speed_res_table.size() != 0){
      try {
        auto msg = msg_result_.ego_speed_res_table.front().get();
        pub_autoware_ego_speed_.publish(msg.second);
      } catch (const std::future_error& e) {
          if (e.code() == std::make_error_code(std::future_errc::no_state)) {
              std::cerr << "No associated state(ego state)" << std::endl;
          } else {
              std::cerr << "Caught std::future_error(ego state): " << e.what() << std::endl;
          }
      } catch (const std::exception& e) {
          std::cerr << "Caught exception(ego state): " << e.what() << std::endl;
      }
      msg_result_.ego_speed_res_table.pop();
    }    

  }
}
