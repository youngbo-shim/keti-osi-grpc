#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/alarm.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include "sensorview_rpc.grpc.pb.h"

#include "morai_msgs/EgoVehicleStatus.h"
#include "morai_msgs/GPSMessage.h"
#include "morai_msgs/CtrlCmd.h"
#include "morai_msgs/MoraiEventCmdSrv.h"
#include "morai_msgs/ObjectStatusList.h"
#include "morai_msgs/GetTrafficLightStatus.h"
#include "sensor_msgs/Imu.h"
#include "math/math_utils.h"
#include "conversion/coordinate/utm_conversion.h"
#include "conversion/coordinate/tm_conversion.h"

using namespace keti::common;

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
using osi3::MountingPosition;

using PhaseTable = std::unordered_map<int, std::vector<osi3::TrafficLight>>;
using TypeTable = std::unordered_map<int, PhaseTable>;
using ObjTable = std::map<std::pair<int, std::string>, osi3::MovingObject::VehicleClassification>;



static constexpr unsigned int Hash(const char* str){
  return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * Hash(str + 1) : 8603;
}

class SensorViewRPCImpl final
{
public:
  explicit SensorViewRPCImpl(std::string address) : address_(address) {    

    XmlRpc::XmlRpcValue camera_param, lidar_param;
    nh_.getParam("camera_param", camera_param);
    nh_.getParam("lidar_param", lidar_param);
    nh_.getParam("data_path", hdmap_path_);

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
  
    camera_buf_.resize(camera_param.size());
    lidar_buf_.resize(lidar_param.size());
    sub_camera_topics_.resize(camera_param.size());
    sub_lidar_topics_.resize(lidar_param.size());
    camera_tf_buf_.resize(camera_param.size());
    lidar_tf_buf_.resize(lidar_param.size());
  
    for (size_t i = 0; i < camera_param.size(); i++){
      sub_camera_topics_[i] = nh_.subscribe<sensor_msgs::CompressedImage>(camera_param[i][0], 1, 
                                                                  boost::bind(&SensorViewRPCImpl::CallbackImage, this, _1, i));
      
      // InitTFTable(camera_param[i][1], i, CAMERA);
    }

    for (size_t i = 0; i < lidar_param.size(); i++){
      sub_lidar_topics_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_param[i][0], 1, 
                                                                  boost::bind(&SensorViewRPCImpl::CallbackPointCloud, this, _1, i));

      // InitTFTable(lidar_param[i][1], i, LIDAR);  
    }

    sub_morai_vehicle_state_ = nh_.subscribe("/Ego_topic", 1, &SensorViewRPCImpl::CallbackVehicleState, this);
    sub_morai_gps_ = nh_.subscribe("/gps", 1, &SensorViewRPCImpl::CallbackGPS, this);
    sub_morai_object_info_ = nh_.subscribe("/Object_topic", 1, &SensorViewRPCImpl::CallbackObjectInfo, this);
    sub_morai_traffic_light_status_ = nh_.subscribe("/GetTrafficLightStatus", 1, &SensorViewRPCImpl::CallbackTrafficLight, this);
    sub_is_changed_offset_ = nh_.subscribe("/is_changed_offset",1, &SensorViewRPCImpl::CallbackUpdateOffsetParams, this);

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
    LIDAR = 2
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
    new CallData(&service_, cq_.get(), &camera_buf_, &lidar_buf_, &camera_tf_buf_, &lidar_tf_buf_,
                 &obj_buf_, &tl_buf_, &morai_tm_offset_, &ego_vehicle_heading_, &hdmap_path_);
    void *tag; // uniquely identifies a request.
    bool ok;

    gpr_timespec deadline;
    deadline.clock_type = GPR_TIMESPAN;
    deadline.tv_sec = 0;
    deadline.tv_nsec = 10000000;

    while (true)
    {
      std::chrono::time_point<std::chrono::system_clock> start =
                                        std::chrono::system_clock::now();
      if(!HasData()) continue;
      cq_->Next(&tag, &ok);

      std::chrono::duration<double> next_running_time = std::chrono::system_clock::now() - start;
      std::cout.precision(2);
      if(next_running_time.count() * 1000 > 10.0){
        std::cout << "next running time : " << next_running_time.count() * 1000 << "ms" << std::endl;
      }
      GPR_ASSERT(ok);

      // std::cout << "[tag, ok] : " << tag << ", " << ok << std::endl;
      if ( !static_cast<CallData *>(tag)->Proceed(ok)){
        break;
      }

      std::chrono::duration<double> running_time = std::chrono::system_clock::now() - start;
      std::cout.precision(3);
      // if (!static_cast<CallData *>(tag)->Proceed(ok))
      //   break;
    }

    std::cout << "thread join" << std::endl;
  }

  void CallbackImage(const sensor_msgs::CompressedImageConstPtr& img, size_t idx){
    std::lock_guard<std::mutex> lock(camera_mutex_);
    std::cout << "start push img " << idx << std::endl;
    camera_buf_[idx].push_back(img);
    std::cout << "finish push img " << idx << std::endl;
  }

  void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud, size_t idx){
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buf_[idx].push_back(cloud);
  }

  void CallbackVehicleState(const morai_msgs::EgoVehicleStatus& vehicle_state){
    ego_vehicle_heading_ = math::deg2rad(vehicle_state.heading);
  }

  void CallbackGPS(const morai_msgs::GPSMessage& msg){
    morai_tm_offset_[0] = msg.eastOffset;
    morai_tm_offset_[1] = msg.northOffset;
  }

  void CallbackObjectInfo(const morai_msgs::ObjectStatusListPtr& objs) {
    obj_buf_.push(objs);
  }

  void CallbackTrafficLight(const morai_msgs::GetTrafficLightStatusPtr& traffic_light_status){
    tl_buf_.push(traffic_light_status);
  }

  void CallbackUpdateOffsetParams(const std_msgs::Bool& check) {  
    std::cout << "update offset" << std::endl;
    nh_.getParam("data_path", hdmap_path_);
  }

  bool HasData(){
    for(auto camera_buf : camera_buf_){
      if(!camera_buf.empty()) return true;
    }

    for(auto lidar_buf : lidar_buf_){
      if(!lidar_buf.empty()) return true;
    }

    if(!tl_buf_.empty()) return true;

    if(!obj_buf_.empty()) return true;
    
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
            std::vector<std::list<sensor_msgs::CompressedImageConstPtr>> *camera_buf,
            std::vector<std::list<sensor_msgs::PointCloud2ConstPtr>> *lidar_buf,
            std::vector<MountingPosition> *camera_tf_buf,
            std::vector<MountingPosition> *lidar_tf_buf, 
            std::queue<morai_msgs::ObjectStatusListPtr> *obj_buf,
            std::queue<morai_msgs::GetTrafficLightStatusPtr> *tl_buf,
            double(*morai_tm_offset)[2], double* ego_vehicle_heading, std::string* hdmap_path)
        : service_(service), cq_(cq), camera_buf_(camera_buf), lidar_buf_(lidar_buf), 
          camera_tf_buf_(camera_tf_buf), lidar_tf_buf_(lidar_tf_buf),
          obj_buf_(obj_buf), tl_buf_(tl_buf), morai_tm_offset_(morai_tm_offset),
          ego_vehicle_heading_(ego_vehicle_heading), hdmap_path_(hdmap_path), responder_(&ctx_), status_(CREATE)
    {
      num_writing_ = 0;

      // Init table
      TrafficLightIdMathching();
      TrafficLightMoraiToOSIMatching();
      ObjectMoraiToOSIMatching();

      // Invoke the serving logic right away.
      Proceed(true);
    }

    void TrafficLightIdMathching(){
      std::string csv_path, line, morai_id, keti_id, temp_string;         
      std::ifstream fs; 
      csv_path = ros::package::getPath("async_stream") + "/src/matching_table/id_matching_test.csv";
      fs.open(csv_path, std::ios::in);

      if (!fs.is_open()) {
        std::cout << "can not open file!" << std::endl;
        return;
      }

      while(getline(fs, line)){
        std::stringstream line_str(line);
        for( int i = 0 ; i < 2 ; i++){
          getline(line_str, temp_string, '/');
          if(i == 0){
            morai_id = temp_string;
            // cout << "morai_id: " << morai_id << endl;
          }
          if(i == 1){
            std::stringstream keti_id_str(temp_string);
            while(getline(keti_id_str, keti_id, ',')){
              id_table_[morai_id].push_back(keti_id);
              // cout << "keti id : " << keti_id << endl;
            }
          }
        }
        // cout << "\n";
      }
      fs.close();
    }

    void TrafficLightMoraiToOSIMatching(){
      std::string csv_path, line, morai_signal_phase, bulb_mode, temp_string;         
      int type, phase;
      bool phase_read_start = true;
      PhaseTable phase_table;
      std::vector<osi3::TrafficLight> osi_tls;
      std::ifstream fs; 
      osi3::TrafficLight tl_1, tl_2, tl_3, tl_4;

      tl_1.mutable_id()->set_value(1);
      tl_1.mutable_classification()->set_color(osi3::TrafficLight::Classification::COLOR_RED);
      tl_1.mutable_classification()->set_icon(osi3::TrafficLight::Classification::ICON_NONE);
      tl_2.mutable_id()->set_value(2);
      tl_2.mutable_classification()->set_color(osi3::TrafficLight::Classification::COLOR_YELLOW);
      tl_2.mutable_classification()->set_icon(osi3::TrafficLight::Classification::ICON_NONE);
      tl_3.mutable_id()->set_value(3);
      tl_3.mutable_classification()->set_color(osi3::TrafficLight::Classification::COLOR_GREEN);
      tl_3.mutable_classification()->set_icon(osi3::TrafficLight::Classification::ICON_ARROW_LEFT);
      tl_4.mutable_id()->set_value(4);
      tl_4.mutable_classification()->set_color(osi3::TrafficLight::Classification::COLOR_GREEN);
      tl_4.mutable_classification()->set_icon(osi3::TrafficLight::Classification::ICON_NONE);

      csv_path = ros::package::getPath("async_stream") + "/src/matching_table/morai_to_osi_matching_tl.csv";
      fs.open(csv_path, std::ios::in);

      if (!fs.is_open()) {
        std::cout << "can not open file!" << std::endl;
        return;
      }

      while(getline(fs, line)){
        if(line.compare("end") == 0){
          phase_read_start = true;
          morai_to_osi_matching_table_tl_[type] = phase_table;
          phase_table.clear();
          osi_tls.clear();
          continue;
        }
        if(phase_read_start == true){
          type = std::stoi(line);
          phase_read_start = false;
          if(type == 0){
            osi_tls.push_back(tl_1);
            osi_tls.push_back(tl_2);
            osi_tls.push_back(tl_4);
          }
          else if (type == 1 || type == 2){
            osi_tls.push_back(tl_1);
            osi_tls.push_back(tl_2);
            osi_tls.push_back(tl_3);
            osi_tls.push_back(tl_4);
          }
          continue;
        }
        std::stringstream line_str(line);
        for( int i = 0 ; i < 2 ; i++){
          getline(line_str, temp_string, ':');
          if(i == 0){
            phase = std::stoi(temp_string);
            phase_table[phase] = osi_tls;
          }
          if(i == 1){
            std::stringstream traffic_light_id_str(temp_string);
            int bulb_count = 0;
            while(getline(traffic_light_id_str, bulb_mode, ',')){
              switch(Hash(bulb_mode.c_str())){
                case Hash("OTHER"):
                  phase_table[phase][bulb_count].mutable_classification()->set_mode(osi3::TrafficLight::Classification::MODE_OTHER);
                  break;
                case Hash("OFF"):
                  phase_table[phase][bulb_count].mutable_classification()->set_mode(osi3::TrafficLight::Classification::MODE_OFF);
                  break;
                case Hash("CONSTANT"):
                  phase_table[phase][bulb_count].mutable_classification()->set_mode(osi3::TrafficLight::Classification::MODE_CONSTANT);
                  break;
                case Hash("FLASHING"):
                  phase_table[phase][bulb_count].mutable_classification()->set_mode(osi3::TrafficLight::Classification::MODE_FLASHING);
                  break;
                case Hash("COUNTING"):
                  phase_table[phase][bulb_count].mutable_classification()->set_mode(osi3::TrafficLight::Classification::MODE_COUNTING);
                  break;
                default:
                  break;
              }
              bulb_count++;
            }
            std::cout << "\n";
          }
        }
      }
      fs.close();

      for(auto type : morai_to_osi_matching_table_tl_){
        std::cout << "type : " << type.first << std::endl;
        for(auto phase : type.second){
          std::cout << "phase : " << phase.first << std::endl;
          for(auto osi_tl : phase.second){
            std::cout << "id : " << osi_tl.id().value() << ", icon : " << osi_tl.classification().icon() 
                      << ", color : " << osi_tl.classification().color()
                      << ", mode : " << osi_tl.classification().mode() << std::endl;
          }
        }
      }
    }

    void ObjectMoraiToOSIMatching(){
      std::string csv_path, line, vehicle_name, temp_string;         
      int priority_count = 0;
      osi3::MovingObject::VehicleClassification vehicle_type;
      std::ifstream fs; 

      csv_path = ros::package::getPath("async_stream") + "/src/matching_table/morai_to_osi_matching_obj_2.csv";
      fs.open(csv_path, std::ios::in);

      if (!fs.is_open()) {
        std::cout << "can not open file!" << std::endl;
        return;
      }

      while(getline(fs, line)){
        priority_count++;
        std::stringstream line_str(line);
        for( int i = 0 ; i < 2 ; i++){
          getline(line_str, temp_string, ':');
          if(i == 0){
            switch(Hash(temp_string.c_str())){
              case Hash("bus"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_BUS);
                break;
              case Hash("truck"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_HEAVY_TRUCK);
                break;
              case Hash("medium_vehicle"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_MEDIUM_CAR);
                break;
              case Hash("standup_scooter"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_STANDUP_SCOOTER);
                break;
              case Hash("motorbike"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_MOTORBIKE);
                break;
              case Hash("bicycle"):
                vehicle_type.set_type(osi3::MovingObject::VehicleClassification::TYPE_BICYCLE);
                break;
              default:
                std::cout << "undefined type : " << temp_string << std::endl;
                break;
            }
          }
          if(i == 1){
            std::stringstream vehicle_name_str(temp_string);
            while(getline(vehicle_name_str, vehicle_name, ',')){
              morai_to_osi_matching_table_obj_[std::make_pair(priority_count, vehicle_name)] = vehicle_type;
            }
          }
        }
      }
      fs.close();

      for(auto type : morai_to_osi_matching_table_obj_){
        std::cout << "vehicle name : " << type.first.second << ", vehice type enum : " << type.second.type() << std::endl;
      }
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

          // img_buf_1_->clear();
          // img_buf_2_->clear();
          // cloud_buf_1_->clear();
          // cloud_buf_2_->clear();
        }
        else if (status_ == PROCESS)
        {
          reply_.Clear();          
          has_data_ = false;

          for(int i = 0 ; i < camera_buf_->size() ; i++){
            if(camera_buf_->at(i).size() == 0) continue;
            std::cout << "camera_buf->at(" << i << ").size() : " << camera_buf_->at(i).size() << std::endl;
            auto& sending_img = camera_buf_->at(i).front();
            std::cout << "finish getting img" << std::endl;
            auto camera_view = reply_.add_camera_sensor_view();
            auto camera_view_configuration = camera_view->mutable_view_configuration();
            camera_view->mutable_image_data()->resize(sending_img->data.size());
            *camera_view->mutable_image_data() = {sending_img->data.begin(), sending_img->data.end()};
            camera_view_configuration->mutable_sensor_id()->set_value(i);
            auto mounting_position = camera_tf_buf_->at(i);
            camera_view_configuration->mutable_mounting_position()->mutable_position()->set_x(mounting_position.position().x());
            camera_view_configuration->mutable_mounting_position()->mutable_position()->set_y(mounting_position.position().y());
            camera_view_configuration->mutable_mounting_position()->mutable_position()->set_z(mounting_position.position().z());
            camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(mounting_position.orientation().roll());
            camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(mounting_position.orientation().pitch());
            camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(mounting_position.orientation().yaw());

            std::cout << "start pop img" << std::endl;
            camera_buf_->at(i).pop_front();
            std::cout << "finish pop img" << std::endl;
            has_data_ = true;
          }

          for(int i = 0 ; i < lidar_buf_->size() ; i++){
            if(lidar_buf_->at(i).size() == 0) continue;
            std::cout << "lidar_buf_->at(" << i << ").size() : " << lidar_buf_->at(i).size() << std::endl;
            auto& sending_cloud =  lidar_buf_->at(i).front();

            auto lidar_view = reply_.add_lidar_sensor_view();
            auto lidar_view_configuration = lidar_view->mutable_view_configuration();
            lidar_view_configuration->set_number_of_rays_vertical(64);
            lidar_view_configuration->set_number_of_rays_horizontal(sending_cloud->width/64);
            lidar_view_configuration->set_num_of_pixels(sending_cloud->width*sending_cloud->height);
            lidar_view_configuration->mutable_sensor_id()->set_value(i);
            auto mounting_position = lidar_tf_buf_->at(i);
            lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_x(mounting_position.position().x());
            lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_y(mounting_position.position().y());
            lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_z(mounting_position.position().z());
            lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(mounting_position.orientation().roll());
            lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(mounting_position.orientation().pitch());
            lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(mounting_position.orientation().yaw());

            for (size_t i = 0; i < sending_cloud->data.size();){
              // x
              size_t start_idx = i + sending_cloud->fields[0].offset;
              std::vector<uint8_t> point_buf = 
                        std::vector<uint8_t>(sending_cloud->data.begin() + start_idx,
                                            sending_cloud->data.begin() + start_idx + 4);

              float x_value = 0.0;
              assert(point_buf.size() == sizeof(float));
              memcpy(&x_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));

              // y
              start_idx = i + sending_cloud->fields[1].offset;
              point_buf = std::vector<uint8_t>(sending_cloud->data.begin() + start_idx,
                                              sending_cloud->data.begin() + start_idx + 4);

              float y_value = 0.0;
              assert(point_buf.size() == sizeof(float));
              memcpy(&y_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));

              // z
              start_idx = i + sending_cloud->fields[2].offset;
              point_buf = std::vector<uint8_t>(sending_cloud->data.begin() + start_idx,
                                              sending_cloud->data.begin() + start_idx + 4);

              float z_value = 0.0;
              assert(point_buf.size() == sizeof(float));
              memcpy(&z_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));
              
              // intensity
              start_idx = i + sending_cloud->fields[3].offset;
              point_buf = std::vector<uint8_t>(sending_cloud->data.begin() + start_idx,
                                              sending_cloud->data.begin() + start_idx + 4);

              float intensity_value = 0.0;
              assert(point_buf.size() == sizeof(float));
              memcpy(&intensity_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));

              double magnitude_direction = std::sqrt(x_value*x_value + y_value*y_value + z_value*z_value);              
              
              // Add direction
              auto direction = lidar_view_configuration->add_directions();
              direction->set_x(x_value/magnitude_direction);
              direction->set_y(y_value/magnitude_direction);
              direction->set_z(z_value/magnitude_direction);

              int speed_of_light = 299792458; // m/s
              auto reflection = lidar_view->add_reflection();
              reflection->set_time_of_flight(magnitude_direction/speed_of_light);
              reflection->set_signal_strength(intensity_value);

              i += sending_cloud->fields.back().offset + 4; // 4 is the size of float
            }

            lidar_buf_->at(i).pop_front();

          //   // auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);
          //   // std::cout << "Running time to make lidar data : " << (double)time_diff.count() << "ms" << std::endl;
          //   // std::chrono::duration<double> running_time = std::chrono::system_clock::now() - start;
          //   // std::cout.precision(3);
          //   // std::cout << "converting running time : " << running_time.count() * 1000 << "ms" << std::endl;
            has_data_ = true;
          }

          if(tl_buf_->size() > 0){
            std::cout << "tl_buf_->size() : " << tl_buf_->size() << std::endl;
            auto& sending_tl = tl_buf_->front();

            auto osi_tls = morai_to_osi_matching_table_tl_[sending_tl->trafficLightType][sending_tl->trafficLightStatus];

            for(auto osi_tl : osi_tls){
              auto tl = reply_.mutable_global_ground_truth()->add_traffic_light();
              tl->mutable_id()->set_value(osi_tl.id().value());
              tl->mutable_classification()->set_color(osi_tl.classification().color());
              tl->mutable_classification()->set_icon(osi_tl.classification().icon());
              tl->mutable_classification()->set_mode(osi_tl.classification().mode());
              tl->add_source_reference()->add_identifier(sending_tl->trafficLightIndex);
            }

            // for(auto osi_tl : reply_.global_ground_truth().traffic_light()){
            //   std::cout << "id : " << osi_tl.id().value() << ", icon : " << osi_tl.classification().icon() 
            //             << ", color : " << osi_tl.classification().color()
            //             << ", mode : " << osi_tl.classification().mode() 
            //             << ", morai tl id : " << osi_tl.source_reference(0).identifier(0) << std::endl;
            // }

            tl_buf_->pop();
            has_data_ = true;
          }

          if(obj_buf_->size() > 0){
            std::cout << "obj_buf_->size() : " << obj_buf_->size() << std::endl;
            auto& sending_obj = obj_buf_->front();

            std::cout << "hdmap path : " << *hdmap_path_ << std::endl;

            CGeoCoordConv geo_conv;
            double x_utm, y_utm, x_vel, y_vel, x_accel, y_accel;
            if(hdmap_path_->substr(hdmap_path_->rfind("/")+1) == "Pangyo HD Map"){
              geo_conv = CGeoCoordConv(GeoEllips::kGrs80, GeoSystem::kTmMid,
                                      GeoEllips::kWgs84, GeoSystem::kUtm52);
            }

            else if(hdmap_path_->substr(hdmap_path_->rfind("/")+1) == "KATRI HD Map"){
              geo_conv = CGeoCoordConv(GeoEllips::kWgs84, GeoSystem::kUtm52,
                                      GeoEllips::kWgs84, GeoSystem::kUtm52);
            }

            if(sending_obj->npc_list.size() == 0 && sending_obj->pedestrian_list.size() == 0 && sending_obj->obstacle_list.size() == 0){
              auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
              auto stationary_obj = reply_.mutable_global_ground_truth()->add_stationary_object();

              moving_obj->mutable_id()->set_value(0);
              stationary_obj->mutable_id()->set_value(0);
            }

            std::vector<osi3::MovingObject> moving_objs;
            std::vector<osi3::StationaryObject> stationary_objs;

            double ego_heading = *ego_vehicle_heading_;
            double morai_tm_x_offset = (*morai_tm_offset_)[0];
            double morai_tm_y_offset = (*morai_tm_offset_)[1];

            for(auto morai_obj : sending_obj->npc_list){
              auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
              moving_obj->mutable_id()->set_value(morai_obj.unique_id);
              moving_obj->set_type(osi3::MovingObject::TYPE_VEHICLE);
              moving_obj->mutable_vehicle_classification()->set_type(osi3::MovingObject::VehicleClassification::TYPE_MEDIUM_CAR);
              // vehicle_type
              for(auto search_obj : morai_to_osi_matching_table_obj_){
                if(search_obj.second.type() != osi3::MovingObject::VehicleClassification::TYPE_BUS &&
                  search_obj.second.type() != osi3::MovingObject::VehicleClassification::TYPE_HEAVY_TRUCK){
                  continue;
                }
                if(morai_obj.name.find(search_obj.first.second) != std::string::npos){
                  moving_obj->mutable_vehicle_classification()->set_type(search_obj.second.type());
                  break;
                }
              }
              // name
              moving_obj->add_source_reference()->add_identifier(morai_obj.name);
              // size (m)
              moving_obj->mutable_base()->mutable_dimension()->set_length(morai_obj.size.x);
              moving_obj->mutable_base()->mutable_dimension()->set_width(morai_obj.size.y);
              moving_obj->mutable_base()->mutable_dimension()->set_height(morai_obj.size.z);

              // position
              geo_conv.Conv(morai_obj.position.x + morai_tm_x_offset,
                            morai_obj.position.y + morai_tm_y_offset,
                            x_utm, y_utm);
              moving_obj->mutable_base()->mutable_position()->set_x(x_utm);
              moving_obj->mutable_base()->mutable_position()->set_y(y_utm);
              // moving_obj->mutable_base()->mutable_position()->set_z(morai_obj.position.z);

              //heading (degree to rad)
              double obj_heading = math::deg2rad(morai_obj.heading);
              moving_obj->mutable_base()->mutable_orientation()->set_yaw(obj_heading);

              // To Ego vehicle coordinate
              // velocity
              double local_heading = obj_heading - ego_heading;

              double velocity_2d[2] = {math::kmh2ms(morai_obj.velocity.x), math::kmh2ms(morai_obj.velocity.y)};
              x_vel = cos(local_heading)*velocity_2d[0]
                    - sin(local_heading)*velocity_2d[1];
              y_vel = sin(local_heading)*velocity_2d[0]
                    + cos(local_heading)*velocity_2d[1];

              // acceleration
              double accel_2d[2] = {morai_obj.acceleration.x, morai_obj.acceleration.y};    
              x_accel = cos(local_heading)*accel_2d[0]
                    - sin(local_heading)*accel_2d[1];
              y_accel = sin(local_heading)*accel_2d[0]
                    + cos(local_heading)*accel_2d[1];

              // To Global coordinate
              // velocity
              double tmp_x_vel = x_vel;
              double tmp_y_vel = y_vel;

              x_vel = cos(ego_heading)*tmp_x_vel
                      -sin(ego_heading)*tmp_y_vel;

              y_vel = sin(ego_heading)*tmp_x_vel
                      +cos(ego_heading)*tmp_y_vel;

              moving_obj->mutable_base()->mutable_velocity()->set_x(x_vel);
              moving_obj->mutable_base()->mutable_velocity()->set_y(y_vel);
                                              

              double tmp_x_accel = x_accel;
              double tmp_y_accel = y_accel;

              x_accel = cos(ego_heading)*tmp_x_accel
                        -sin(ego_heading)*tmp_y_accel;

              y_accel = sin(ego_heading)*tmp_x_accel
                        +cos(ego_heading)*tmp_y_accel;

              moving_obj->mutable_base()->mutable_acceleration()->set_x(x_accel);
              moving_obj->mutable_base()->mutable_acceleration()->set_y(y_accel);
            }

            for(auto morai_obj : sending_obj->pedestrian_list){
              auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
              moving_obj->mutable_id()->set_value(morai_obj.unique_id);
              moving_obj->set_type(osi3::MovingObject::TYPE_PEDESTRIAN);
              //name
              moving_obj->add_source_reference()->add_identifier(morai_obj.name);
              //size (m)
              moving_obj->mutable_base()->mutable_dimension()->set_width(morai_obj.size.x);
              moving_obj->mutable_base()->mutable_dimension()->set_length(morai_obj.size.y);
              moving_obj->mutable_base()->mutable_dimension()->set_height(morai_obj.size.z);

              // position
              geo_conv.Conv(morai_obj.position.x + morai_tm_x_offset,
                            morai_obj.position.y + morai_tm_y_offset,
                            x_utm, y_utm);
              moving_obj->mutable_base()->mutable_position()->set_x(x_utm);
              moving_obj->mutable_base()->mutable_position()->set_y(y_utm);
              // moving_obj->mutable_base()->mutable_position()->set_z(morai_obj.position.z);

              //heading (degree to rad)
              double obj_heading = math::deg2rad(morai_obj.heading);
              moving_obj->mutable_base()->mutable_orientation()->set_yaw(obj_heading);

              // To Ego vehicle coordinate
              // velocity
              double local_heading = obj_heading - ego_heading;

              double velocity_2d[2] = {math::kmh2ms(morai_obj.velocity.x), math::kmh2ms(morai_obj.velocity.y)};
              x_vel = cos(local_heading)*velocity_2d[0]
                    - sin(local_heading)*velocity_2d[1];
              y_vel = sin(local_heading)*velocity_2d[0]
                    + cos(local_heading)*velocity_2d[1];

              // acceleration
              double accel_2d[2] = {morai_obj.acceleration.x, morai_obj.acceleration.y};    
              x_accel = cos(local_heading)*accel_2d[0]
                    - sin(local_heading)*accel_2d[1];
              y_accel = sin(local_heading)*accel_2d[0]
                    + cos(local_heading)*accel_2d[1];

              // To Global coordinate
              // velocity
              double tmp_x_vel = x_vel;
              double tmp_y_vel = y_vel;

              x_vel = cos(ego_heading)*tmp_x_vel
                      -sin(ego_heading)*tmp_y_vel;

              y_vel = sin(ego_heading)*tmp_x_vel
                      +cos(ego_heading)*tmp_y_vel;

              moving_obj->mutable_base()->mutable_velocity()->set_x(x_vel);
              moving_obj->mutable_base()->mutable_velocity()->set_y(y_vel);
                                              

              double tmp_x_accel = x_accel;
              double tmp_y_accel = y_accel;

              x_accel = cos(ego_heading)*tmp_x_accel
                        -sin(ego_heading)*tmp_y_accel;

              y_accel = sin(ego_heading)*tmp_x_accel
                        +cos(ego_heading)*tmp_y_accel;

              moving_obj->mutable_base()->mutable_acceleration()->set_x(x_accel);
              moving_obj->mutable_base()->mutable_acceleration()->set_y(y_accel);
            }

            for(auto morai_obj : sending_obj->obstacle_list){
              osi3::MovingObject::VehicleClassification moving_obj_type;
              bool is_stationary_obj = true;
              // type
              for(auto search_obj : morai_to_osi_matching_table_obj_){
                if(morai_obj.name.find(search_obj.first.second) != std::string::npos){
                  // moving_obj_type.set_type(osi3::MovingObject::TYPE_VEHICLE);
                  // moving_obj_type.mutable_vehicle_classification()->set_type(search_obj.second.type());
                  moving_obj_type.set_type(search_obj.second.type());
                  is_stationary_obj = false;
                  break;
                }
              }
              if(is_stationary_obj){
                auto stationary_obj = reply_.mutable_global_ground_truth()->add_stationary_object();
                // id
                stationary_obj->mutable_id()->set_value(morai_obj.unique_id);
                // name
                stationary_obj->add_source_reference()->add_identifier(morai_obj.name);
                // size (m)
                stationary_obj->mutable_base()->mutable_dimension()->set_length(morai_obj.size.x);
                stationary_obj->mutable_base()->mutable_dimension()->set_width(morai_obj.size.y);
                stationary_obj->mutable_base()->mutable_dimension()->set_height(morai_obj.size.z);

                // position
                geo_conv.Conv(morai_obj.position.x + morai_tm_x_offset,
                              morai_obj.position.y + morai_tm_y_offset,
                              x_utm, y_utm);
                stationary_obj->mutable_base()->mutable_position()->set_x(x_utm);
                stationary_obj->mutable_base()->mutable_position()->set_y(y_utm);
                // stationary_obj->mutable_base()->mutable_position()->set_z(morai_obj.position.z);

                //heading (degree to rad)
                stationary_obj->mutable_base()->mutable_orientation()->set_yaw(math::deg2rad(morai_obj.heading));
              }
              else{
                auto moving_obj = reply_.mutable_global_ground_truth()->add_moving_object();
                // id
                moving_obj->mutable_id()->set_value(morai_obj.unique_id);
                // name
                moving_obj->add_source_reference()->add_identifier(morai_obj.name);
                // size (m)
                moving_obj->mutable_base()->mutable_dimension()->set_length(morai_obj.size.x);
                moving_obj->mutable_base()->mutable_dimension()->set_width(morai_obj.size.y);
                moving_obj->mutable_base()->mutable_dimension()->set_height(morai_obj.size.z);
                // type
                moving_obj->set_type(osi3::MovingObject::TYPE_VEHICLE);
                moving_obj->mutable_vehicle_classification()->set_type(moving_obj_type.type());
                // position
                geo_conv.Conv(morai_obj.position.x + morai_tm_x_offset,
                              morai_obj.position.y + morai_tm_y_offset,
                              x_utm, y_utm);
                moving_obj->mutable_base()->mutable_position()->set_x(x_utm);
                moving_obj->mutable_base()->mutable_position()->set_y(y_utm);
                // moving_obj->mutable_base()->mutable_position()->set_z(morai_obj.position.z);

                //heading (degree to rad)
                moving_obj->mutable_base()->mutable_orientation()->set_yaw(math::deg2rad(morai_obj.heading));
              }
            }

            if(sending_obj->npc_list.size() != 0 || sending_obj->pedestrian_list.size() != 0 || sending_obj->obstacle_list.size() != 0){
              for(auto check_test : reply_.global_ground_truth().moving_object()){
                std::cout << "name : " << check_test.source_reference(0).identifier(0)
                          << ", roll : " << check_test.base().orientation().roll()
                          << ", pitch : " << check_test.base().orientation().pitch()
                          << ", yaw : " << check_test.base().orientation().yaw()
                          << ", vel_x : " << check_test.base().velocity().x()
                          << ", vel_y : " << check_test.base().velocity().y()
                          << ", vel_z : " << check_test.base().velocity().z()
                          << ", type : " << check_test.type()
                          << ", sub_type : " << check_test.vehicle_classification().type() << std::endl;
              }

              for(auto check_test : reply_.global_ground_truth().stationary_object()){
                std::cout << "name : " << check_test.source_reference(0).identifier(0)
                          << ", roll : " << check_test.base().orientation().roll()
                          << ", pitch : " << check_test.base().orientation().pitch()
                          << ", yaw : " << check_test.base().orientation().yaw() << std::endl;
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
            std::cout << std::fixed << "[current] : " << current_nanoseconds << std::endl;
            
            std::chrono::duration<double> running_time = std::chrono::system_clock::now() - start;
            std::cout.precision(3);
            std::cout << "converting running time : " << running_time.count() * 1000 << "ms" << std::endl;
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
    TypeTable morai_to_osi_matching_table_tl_;
    ObjTable morai_to_osi_matching_table_obj_;
    std::unordered_map<std::string, std::vector<std::string>> id_table_;

    // Injected data
    std::vector<std::list<sensor_msgs::CompressedImageConstPtr>> *camera_buf_;
    std::vector<std::list<sensor_msgs::PointCloud2ConstPtr>> *lidar_buf_;
    std::vector<MountingPosition> *camera_tf_buf_;
    std::vector<MountingPosition> *lidar_tf_buf_;
    double(*morai_tm_offset_)[2];
    double* ego_vehicle_heading_;
    std::string* hdmap_path_;
    std::queue<morai_msgs::ObjectStatusListPtr> *obj_buf_;
    std::queue<morai_msgs::GetTrafficLightStatusPtr> *tl_buf_;

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

  // Sensors
  std::vector<std::list<sensor_msgs::CompressedImageConstPtr>> camera_buf_;
  std::vector<std::list<sensor_msgs::PointCloud2ConstPtr>> lidar_buf_;
  std::vector<MountingPosition> camera_tf_buf_;
  std::vector<MountingPosition> lidar_tf_buf_;
  std::mutex lidar_mutex_, camera_mutex_;

  // Ground Truth
  double morai_tm_offset_[2];
  double ego_vehicle_heading_;
  std::queue<morai_msgs::ObjectStatusListPtr> obj_buf_;
  std::queue<morai_msgs::GetTrafficLightStatusPtr> tl_buf_;
  std::string hdmap_path_;

  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> sub_camera_topics_;
  std::vector<ros::Subscriber> sub_lidar_topics_;
  ros::Subscriber sub_morai_vehicle_state_, sub_morai_gps_, sub_morai_object_info_, sub_morai_traffic_light_status_, sub_is_changed_offset_;

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
