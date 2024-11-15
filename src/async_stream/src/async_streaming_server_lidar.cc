#include <utils/sensor_data_osi_converter.h>
#include "morai_ros_bridge.h"

using VehicleGearStatus = SensorDataOSIConverter::VehicleGearStatus;
using VehicleMode = SensorDataOSIConverter::VehicleMode;
using VehicleGearAndModeInfo = SensorDataOSIConverter::VehicleGearAndModeInfo;

class SensorViewRPCImpl final
{
public:
  explicit SensorViewRPCImpl(std::string address) : address_(address) {    

    XmlRpc::XmlRpcValue camera_param, lidar_param, radar_param;
    nh_.getParam("camera_param", camera_param);
    nh_.getParam("lidar_param", lidar_param);
    nh_.getParam("radar_param", radar_param);
    nh_.getParam("data_path", hdmap_path_);
    nh_.getParam("bridge_name", bridge_name_);

    std::cout << "camera_param : size = " << camera_param.size() << std::endl;
    if(camera_param.getType() == XmlRpc::XmlRpcValue::TypeArray &&
       camera_param.size() > 0){
      for(int i = 0 ; i < camera_param.size() ; i++){
        for(int j = 0 ; j < camera_param[i].size() ; j++){
          std::cout << camera_param[i][j] << ", ";
        }
        std::cout << "\n";
      }
    }

    std::cout << "lidar_param : size = " << lidar_param.size() << std::endl;
    if(lidar_param.getType() == XmlRpc::XmlRpcValue::TypeArray &&
       lidar_param.size() > 0){
      for(int i = 0 ; i < lidar_param.size() ; i++){
        for(int j = 0 ; j < lidar_param[i].size() ; j++){
          std::cout << lidar_param[i][j] << ", ";
        }
        std::cout << "\n";
      }
    }

    std::cout << "radar_param : size = " << radar_param.size() << std::endl;
    if(radar_param.getType() == XmlRpc::XmlRpcValue::TypeArray &&
       radar_param.size() > 0){
      for(int i = 0 ; i < radar_param.size() ; i++){
        for(int j = 0 ; j < radar_param[i].size() ; j++){
          std::cout << radar_param[i][j] << ", ";
        }
        std::cout << "\n";
      }
    }
  
    camera_buf_.resize(camera_param.size());
    lidar_buf_.resize(lidar_param.size());
    radar_buf_.resize(radar_param.size());

    sub_camera_topics_.resize(camera_param.size());
    sub_lidar_topics_.resize(lidar_param.size());
    sub_radar_topics_.resize(radar_param.size());

    camera_tf_buf_.resize(camera_param.size());
    lidar_tf_buf_.resize(lidar_param.size());
    radar_tf_buf_.resize(radar_param.size());
  
    for (size_t i = 0; i < camera_param.size(); i++){
      sub_camera_topics_[i] = nh_.subscribe<sensor_msgs::CompressedImage>(camera_param[i][0], 1, 
                                                                  boost::bind(&SensorViewRPCImpl::CallbackImage, this, _1, i));
      
      InitTFTable(camera_param[i][1], i, CAMERA);
    }

    for (size_t i = 0; i < lidar_param.size(); i++){
      sub_lidar_topics_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_param[i][0], 1, 
                                                                  boost::bind(&SensorViewRPCImpl::CallbackPointCloud, this, _1, i));

      InitTFTable(lidar_param[i][1], i, LIDAR);  
    }

    for (size_t i = 0; i < radar_param.size(); i++){
      sub_radar_topics_[i] = nh_.subscribe<morai_msgs::RadarDetections>(radar_param[i][0], 1, 
                                                                  boost::bind(&SensorViewRPCImpl::CallbackRadarDetections, this, _1, i));
      // InitTFTable(radar_param[i][1], i, RADAR);
    }

    sub_morai_vehicle_state_ = nh_.subscribe("/Ego_topic", 1, &SensorViewRPCImpl::CallbackVehicleState, this);
    sub_morai_gps_ = nh_.subscribe("/gps_morai", 1, &SensorViewRPCImpl::CallbackGPS, this);
    sub_morai_object_info_ = nh_.subscribe("/Object_topic", 1, &SensorViewRPCImpl::CallbackObjectInfo, this);
    sub_morai_traffic_light_status_ = nh_.subscribe("/GetTrafficLightStatus", 1, &SensorViewRPCImpl::CallbackTrafficLight, this);
    sub_is_changed_offset_ = nh_.subscribe("/is_changed_offset",1, &SensorViewRPCImpl::CallbackUpdateOffsetParams, this);
    sub_morai_imu_ = nh_.subscribe("/imu", 1, &SensorViewRPCImpl::CallbackImu, this);
    sub_autoware_ego_state_ = nh_.subscribe("/Autoware_ego_topic", 1, &SensorViewRPCImpl::CallbackAutowareEgoState, this);
    morai_ctrl_cli_ = nh_.serviceClient<morai_msgs::MoraiEventCmdSrv>("/Service_MoraiEventCmd");
    vehicle_spec_timer_ = nh_.createTimer(ros::Duration(0.1), &SensorViewRPCImpl::MoraiVehicleCtrlCallback, this);
    is_changed_params_ = false;

    for(int i = 0 ; i < camera_tf_buf_.size() ; i++){
      auto tf = camera_tf_buf_[i];
      
      std::cout << "camera " << i << " tf:\n"
                << " x : " << tf.position().x()
                << " y : " << tf.position().y()
                << " z : " << tf.position().z()
                << " roll : " << tf.orientation().roll()*180.0/M_PI
                << " pitch : " << tf.orientation().pitch()*180.0/M_PI
                << " yaw : " << tf.orientation().yaw()*180.0/M_PI << std::endl;
    }

    for(int i = 0 ; i < lidar_tf_buf_.size() ; i++){
      auto tf = lidar_tf_buf_[i];
      
      std::cout << "lidar " << i << " tf:\n"
                << " x : " << tf.position().x()
                << " y : " << tf.position().y()
                << " z : " << tf.position().z()
                << " roll : " << tf.orientation().roll()*180.0/M_PI
                << " pitch : " << tf.orientation().pitch()*180.0/M_PI
                << " yaw : " << tf.orientation().yaw()*180.0/M_PI << std::endl;
    }
  }

  ~SensorViewRPCImpl()
  {
    server_->Shutdown();
    cq_->Shutdown();
  }

  enum SensorType{
    CAMERA = 1,
    LIDAR = 2,
    RADAR = 3
  };

  void InitTFTable(std::string sensor_frame_id, size_t idx, int type){
    tf::TransformListener listener;
    tf::StampedTransform transform_baselink_to_sensor;  

    bool is_init_trans = false;
    while(!is_init_trans){
      try{
        listener.waitForTransform("/base_link", sensor_frame_id,
                                  ros::Time::now(), ros::Duration(1.0));
        listener.lookupTransform("/base_link", sensor_frame_id,
                                  ros::Time(0), transform_baselink_to_sensor);
        
        MountingPosition tf;
        double roll, pitch, yaw;

        tf.mutable_position()->set_x(transform_baselink_to_sensor.getOrigin().x());
        tf.mutable_position()->set_y(transform_baselink_to_sensor.getOrigin().y());
        tf.mutable_position()->set_z(transform_baselink_to_sensor.getOrigin().z());

        tf::Matrix3x3(transform_baselink_to_sensor.getRotation()).getRPY(roll,pitch,yaw);

        tf.mutable_orientation()->set_roll(roll);
        tf.mutable_orientation()->set_pitch(pitch);
        tf.mutable_orientation()->set_yaw(yaw);

        if(type == CAMERA){
          camera_tf_buf_[idx] = tf;
        }

        if(type == LIDAR){
          lidar_tf_buf_[idx] = tf;
        }

        if(type == RADAR){
          radar_tf_buf_[idx] = tf;
        }

        is_init_trans = true;  
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
      }
    }
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
    builder.SetMaxSendMessageSize(1 * 1024 * 1024 * 1024);
    server_ = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;
  }

  // This can be run in multiple threads if needed.
  void HandleRpcs()
  {
    // Spawn a new CallData instance to serve new clients.
    new CallData(&service_, cq_.get(), &camera_buf_, &lidar_buf_, &radar_buf_,
                 &camera_tf_buf_, &lidar_tf_buf_, &radar_tf_buf_,
                //  &obj_buf_, &tl_buf_, &ego_vehicle_state_buf_, &imu_msg_, &autoware_vehicle_state_,
                 &obj_buf_, &tl_buf_, &ego_vehicle_state_buf_, &imu_msg_, &gps_msg_, &autoware_vehicle_state_, &vehicle_gear_and_mode_info_,
                 &morai_tm_offset_, &ego_vehicle_heading_, &hdmap_path_);
    void *tag; // uniquely identifies a request.
    bool ok;

    gpr_timespec deadline;
    deadline.clock_type = GPR_TIMESPAN;
    deadline.tv_sec = 0;
    deadline.tv_nsec = 10000000;

    while (true)
    {
      if(!HasData()) continue;
      cq_->Next(&tag, &ok);

      // GPR_ASSERT(ok);
      if (!ok) continue;

      if ( !static_cast<CallData *>(tag)->Proceed(ok)){
        break;
      }

      // if (!static_cast<CallData *>(tag)->Proceed(ok))
      //   break;
    }

    std::cout << "thread join" << std::endl;
  }

  void CallbackImage(const sensor_msgs::CompressedImageConstPtr& img, size_t idx){
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_buf_[idx].push(img);
  }

  void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud, size_t idx){
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buf_[idx].push(cloud);
  }

  void CallbackRadarDetections(const morai_msgs::RadarDetectionsConstPtr& detections, size_t idx){
    std::lock_guard<std::mutex> lock(radar_mutex_);
    radar_buf_[idx].push(*detections);
  }

  void CallbackVehicleState(const morai_msgs::EgoVehicleStatusConstPtr& vehicle_state){
    ego_vehicle_state_buf_.push(vehicle_state);
  }

  void CallbackGPS(const morai_msgs::GPSMessagePtr& gps){
    morai_tm_offset_[0] = gps->eastOffset;
    morai_tm_offset_[1] = gps->northOffset;
    gps_msg_ = *gps;
    if(!bridge_name_.compare("apollo")){

      morai_tm_offset_[0] = 267262.707;
      morai_tm_offset_[1] = 3709764.8995;
      // offset from juju map origin (converted from lat, lng)
      // morai_tm_offset_[0] = 267255.863;
      // morai_tm_offset_[1] = 3709764.154;
    }
    else{
      morai_tm_offset_[0] = gps->eastOffset;
      morai_tm_offset_[1] = gps->northOffset;
    }
    if(!nh_.hasParam("x_offset") || !nh_.hasParam("y_offset")){
      nh_.setParam("x_offset", morai_tm_offset_[0]);
      nh_.setParam("y_offset", morai_tm_offset_[1]);
    }
    if(is_changed_params_){      
      nh_.setParam("x_offset", morai_tm_offset_[0]);
      nh_.setParam("y_offset", morai_tm_offset_[1]);
      is_changed_params_ = false;
    }

    // std::cout << "morai_tm_offset_: " << morai_tm_offset_[0] << " " << morai_tm_offset_[1] << std::endl;
  }

  void CallbackImu( const sensor_msgs::ImuConstPtr& imu ){
    imu_msg_ = *imu;
  }

  void CallbackAutowareEgoState( const autoware_msgs::VehicleStatusConstPtr& ego_state ){
    autoware_vehicle_state_ = *ego_state;
  }

  void CallbackObjectInfo(const morai_msgs::ObjectStatusListPtr& objs) {
    obj_buf_.push(objs);
  }

  void CallbackTrafficLight(const morai_msgs::GetTrafficLightStatusPtr& traffic_light_status){
    tl_buf_.push(traffic_light_status);
  }

  void CallbackUpdateOffsetParams(const std_msgs::Bool& check) {  
    nh_.getParam("data_path", hdmap_path_);
    std::cout << "update offset : " << hdmap_path_ << std::endl;
  }

  void MoraiVehicleCtrlCallback(const ros::TimerEvent&) {
    morai_msgs::MoraiEventCmdSrv morai_ctrl_srv;
    morai_ctrl_srv.request.request.option = 0; 
    
    if ( morai_ctrl_cli_.call(morai_ctrl_srv)){
      if ( morai_ctrl_srv.response.response.ctrl_mode == 3 ){ // auto mode
        vehicle_gear_and_mode_info_.vehicle_mode = VehicleMode::AUTO;
      }
      else{
        vehicle_gear_and_mode_info_.vehicle_mode = VehicleMode::MANUAL;
      }

      if ( morai_ctrl_srv.response.response.gear == 1 ){ // PARKING
        vehicle_gear_and_mode_info_.vehicle_gear_status = VehicleGearStatus::PARKING;
      }
      else if ( morai_ctrl_srv.response.response.gear == 2 ){ // REVERSE
        vehicle_gear_and_mode_info_.vehicle_gear_status = VehicleGearStatus::REVERSE;
      }
      else if ( morai_ctrl_srv.response.response.gear == 3 ){ // NEUTRAL
        vehicle_gear_and_mode_info_.vehicle_gear_status = VehicleGearStatus::NEUTRAL;
      }
      else{ // DRIVE
        vehicle_gear_and_mode_info_.vehicle_gear_status = VehicleGearStatus::DRIVE;
      }
    }
  }
    
  bool HasData(){
    for(auto& camera_buf : camera_buf_){
      if(!camera_buf.empty()) return true;
    }

    for(auto& lidar_buf : lidar_buf_){
      if(!lidar_buf.empty()) return true;
    }

    for(auto& radar_buf : radar_buf_){
      if(!radar_buf.empty()) return true;
    }

    if(!tl_buf_.empty()) return true;

    if(!obj_buf_.empty()) return true;

    if(!ego_vehicle_state_buf_.empty()) return true;
    
    return false;
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
            std::vector<std::queue<sensor_msgs::CompressedImageConstPtr>> *camera_buf,
            std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr>> *lidar_buf,
            std::vector<std::queue<morai_msgs::RadarDetections>> *radar_buf,
            std::vector<MountingPosition> *camera_tf_buf,
            std::vector<MountingPosition> *lidar_tf_buf, 
            std::vector<MountingPosition> *radar_tf_buf,
            std::queue<morai_msgs::ObjectStatusListConstPtr> *obj_buf,
            std::queue<morai_msgs::GetTrafficLightStatusPtr> *tl_buf,
            std::queue<morai_msgs::EgoVehicleStatusConstPtr> *ego_vehicle_state_buf,
            sensor_msgs::Imu *imu_msg,
            morai_msgs::GPSMessage *gps_msg,
            autoware_msgs::VehicleStatus *autoware_vehicle_state,
            VehicleGearAndModeInfo *vehicle_gear_and_mode_info,
            double(*morai_tm_offset)[2], double* ego_vehicle_heading, std::string* hdmap_path)
        : service_(service), cq_(cq), camera_buf_(camera_buf), lidar_buf_(lidar_buf), radar_buf_(radar_buf),
          camera_tf_buf_(camera_tf_buf), lidar_tf_buf_(lidar_tf_buf), radar_tf_buf_(radar_tf_buf),
          obj_buf_(obj_buf), tl_buf_(tl_buf), ego_vehicle_state_buf_(ego_vehicle_state_buf), imu_msg_(imu_msg), gps_msg_(gps_msg), autoware_vehicle_state_(autoware_vehicle_state),
          vehicle_gear_and_mode_info_(vehicle_gear_and_mode_info), morai_tm_offset_(morai_tm_offset), ego_vehicle_heading_(ego_vehicle_heading), hdmap_path_(hdmap_path), responder_(&ctx_), status_(CREATE)
    {
      num_writing_ = 0;
      sensor_data_osi_converter_ = std::make_shared<SensorDataOSIConverter>();
      
      // Init table
      sensor_data_osi_converter_->TrafficLightIdMathching();
      sensor_data_osi_converter_->TrafficLightMoraiToOSIMatching();
      sensor_data_osi_converter_->ObjectMoraiToOSIMatching();
      sensor_data_osi_converter_->SetGeoCoordConv(hdmap_path_);
      prev_hdmap_path_ = *hdmap_path_;

      // Invoke the serving logic right away.
      Proceed(true);
    }

    bool Proceed(bool ok)
    {
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

          if ( prev_hdmap_path_ != *hdmap_path_) {
            sensor_data_osi_converter_->SetGeoCoordConv(hdmap_path_);
            prev_hdmap_path_ = *hdmap_path_;
          }

          for(int i = 0 ; i < camera_buf_->size() ; i++){            
            if(camera_buf_->at(i).size() == 0) continue;
            const auto& sending_img = camera_buf_->at(i).front();
            auto camera_view = reply_.add_camera_sensor_view();
            sensor_data_osi_converter_->CameraSensorToOSI(sending_img, camera_view, camera_tf_buf_->at(i), i);
            camera_buf_->at(i).pop();
            has_data_ = true;
          }

          for(int i = 0 ; i < lidar_buf_->size() ; i++){
            if(lidar_buf_->at(i).size() == 0) continue;
            const auto& sending_cloud =  lidar_buf_->at(i).front();
            auto lidar_view = reply_.add_lidar_sensor_view();
            sensor_data_osi_converter_->LidarSensorToOSI(sending_cloud, lidar_view, lidar_tf_buf_->at(i), 64, i);
            lidar_buf_->at(i).pop();
            has_data_ = true;
          }

          for(int i = 0; i < radar_buf_->size() ; i++){
            if (radar_buf_->at(i).size() == 0) continue;
            const auto& sending_detections = radar_buf_->at(i).front();
            auto radar_view = reply_.add_radar_sensor_view();
            sensor_data_osi_converter_->RadarSensorToOSI(sending_detections, radar_view, radar_tf_buf_->at(i), i);
            radar_buf_->at(i).pop();
            has_data_= true;
          }

          if(tl_buf_->size() > 0){
            auto& sending_tl = tl_buf_->front();
            auto osi_tls = sensor_data_osi_converter_->GetMoraiToOSITrafficLightMatchingTable()[sending_tl->trafficLightType][sending_tl->trafficLightStatus];
            for(auto osi_tl : osi_tls){
              auto tl = reply_.mutable_global_ground_truth()->add_traffic_light();
              sensor_data_osi_converter_->TrafficLightsToOSI(sending_tl, tl, osi_tl);
            }
            tl_buf_->pop();
            has_data_ = true;
          }

          if(ego_vehicle_state_buf_->size() > 0){
            const auto& sending_ego_state = ego_vehicle_state_buf_->front();
            auto host_vehicle_view = reply_.mutable_host_vehicle_data();
            if(sensor_data_osi_converter_->GetNeedRealTimeOffsetCal())
              sensor_data_osi_converter_->CalculateMoraiUtmOffset(sending_ego_state, *gps_msg_);
            else
              sensor_data_osi_converter_->SetMoraiTMOffset(morai_tm_offset_);
            sensor_data_osi_converter_->EgoVehicleStateToOSI(sending_ego_state, *imu_msg_, *gps_msg_, *autoware_vehicle_state_, *vehicle_gear_and_mode_info_, host_vehicle_view);
            // sensor_data_osi_converter_->EgoVehicleStateToOSI(sending_ego_state, *imu_msg_, *autoware_vehicle_state_, host_vehicle_view);
            sensor_data_osi_converter_->SetEgoVehicleHeading(host_vehicle_view->vehicle_motion().orientation().yaw() * 180.0 / M_PI);
            ego_vehicle_state_buf_->pop();
            has_data_ = true;
          }

          if(obj_buf_->size() > 0){
            auto& sending_obj = obj_buf_->front();

            if(sending_obj->npc_list.size() == 0 && sending_obj->pedestrian_list.size() == 0 && sending_obj->obstacle_list.size() == 0){
              auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
              auto stationary_obj = reply_.mutable_global_ground_truth()->add_stationary_object();
              sensor_data_osi_converter_->EmptyObstacleToOSI(moving_obj, stationary_obj);
            }

            else{
              for(auto morai_npc_obj : sending_obj->npc_list){
                auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
                sensor_data_osi_converter_->NPCObstacleToOSI(morai_npc_obj, moving_obj);
              }

              for(auto morai_ped_obj : sending_obj->pedestrian_list){
                auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
                sensor_data_osi_converter_->PedObstacleToOSI(morai_ped_obj, moving_obj);
              }

              for(auto morai_obs_obj : sending_obj->obstacle_list){
                osi3::MovingObject::VehicleClassification moving_obj_type;
                bool is_stationary_obj = true;
                // type
                for(auto search_obj : sensor_data_osi_converter_->GetMoraiToOSIObstacleMatchingTable()){
                  if(morai_obs_obj.name.find(search_obj.first.second) != std::string::npos){
                    // moving_obj_type.set_type(osi3::MovingObject::TYPE_VEHICLE);
                    // moving_obj_type.mutable_vehicle_classification()->set_type(search_obj.second.type());
                    moving_obj_type.set_type(search_obj.second.type());
                    is_stationary_obj = false;
                    break;
                  }
                }
                if(is_stationary_obj){
                  auto stationary_obj = reply_.mutable_global_ground_truth()->add_stationary_object();
                  sensor_data_osi_converter_->StaticObstacleToOSI(morai_obs_obj, stationary_obj);
                }
                else{
                  auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
                  sensor_data_osi_converter_->NPCObstacleToOSI(morai_obs_obj,  moving_obj);
                }                
              }
            }

            obj_buf_->pop();
            has_data_ = true;
          }

          if ( has_data_){
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

    // matching table
    // TypeTable morai_to_osi_matching_table_tl_;
    // ObjTable morai_to_osi_matching_table_obj_;
    // std::unordered_map<std::string, std::vector<std::string>> id_table_;

    // Injected data
    std::vector<std::queue<sensor_msgs::CompressedImageConstPtr>> *camera_buf_;
    std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr>> *lidar_buf_;
    std::vector<std::queue<morai_msgs::RadarDetections>> *radar_buf_;
    std::vector<MountingPosition> *camera_tf_buf_;
    std::vector<MountingPosition> *lidar_tf_buf_;
    std::vector<MountingPosition> *radar_tf_buf_;
    double(*morai_tm_offset_)[2];
    double* ego_vehicle_heading_;
    std::string* hdmap_path_;
    std::string prev_hdmap_path_;
    std::queue<morai_msgs::ObjectStatusListConstPtr> *obj_buf_;
    std::queue<morai_msgs::EgoVehicleStatusConstPtr> *ego_vehicle_state_buf_;
    std::queue<morai_msgs::GetTrafficLightStatusPtr> *tl_buf_;
    sensor_msgs::Imu *imu_msg_;
    morai_msgs::GPSMessage *gps_msg_;
    autoware_msgs::VehicleStatus *autoware_vehicle_state_;
    VehicleGearAndModeInfo *vehicle_gear_and_mode_info_;

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
    std::shared_ptr<SensorDataOSIConverter> sensor_data_osi_converter_;
  };

private:
  std::unique_ptr<ServerCompletionQueue> cq_;
  SensorViewRPC::AsyncService service_;
  std::unique_ptr<Server> server_;
  std::string address_;

  // Sensors
  std::vector<std::queue<sensor_msgs::CompressedImageConstPtr>> camera_buf_;
  std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr>> lidar_buf_;
  std::vector<std::queue<morai_msgs::RadarDetections>> radar_buf_;
  std::vector<MountingPosition> camera_tf_buf_, lidar_tf_buf_, radar_tf_buf_;
  std::mutex lidar_mutex_, camera_mutex_, radar_mutex_;

  // Ground Truth
  double morai_tm_offset_[2];
  double ego_vehicle_heading_;
  std::queue<morai_msgs::ObjectStatusListConstPtr> obj_buf_;
  std::queue<morai_msgs::EgoVehicleStatusConstPtr> ego_vehicle_state_buf_;
  std::queue<morai_msgs::GetTrafficLightStatusPtr> tl_buf_;
  sensor_msgs::Imu imu_msg_;
  morai_msgs::GPSMessage gps_msg_;
  autoware_msgs::VehicleStatus autoware_vehicle_state_;
  
  std::string hdmap_path_;
  std::string bridge_name_;

  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> sub_camera_topics_, sub_lidar_topics_, sub_radar_topics_;
  ros::Subscriber sub_morai_vehicle_state_, sub_morai_gps_, sub_morai_object_info_, sub_morai_traffic_light_status_,
                  sub_is_changed_offset_, sub_morai_imu_, sub_autoware_ego_state_;

  ros::ServiceClient morai_ctrl_cli_;
  ros::Timer vehicle_spec_timer_;
  
  bool is_changed_params_;

  VehicleGearAndModeInfo vehicle_gear_and_mode_info_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "morai_keti_bridge_grpc");

  std::string host_name, cmd_host_name;

  host_name = argv[1];
  cmd_host_name = argv[2];

  std::cout << "host_name : " << host_name << std::endl;
  std::cout << "cmd_host_name : " << cmd_host_name << std::endl;

  SensorViewRPCImpl server(host_name);
  server.Run();

  std::thread thread(&SensorViewRPCImpl::HandleRpcs, &server);
  OSIBridge *osi_bridge = new MoraiROSBridge(cmd_host_name, "");

  osi_bridge->ClientStartListen();
  osi_bridge->StartBridge();

  // ros::spin();
  thread.join();

  return 0;
}
