#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/TrafficLights.h"
#include "perception_msgs/TrafficLight.h"
#include "perception_msgs/TrafficSignalPhase.h"

#include "sensorview_rpc.grpc.pb.h"
#include "task/task.h"
#include "hdmap.h"
#include "osi_bridge.cc"

using google::protobuf::RepeatedPtrField;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;

using namespace keti::hdmap;

// static constexpr unsigned int Hash(const char* str){
//   return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * Hash(str + 1) : 8603;
// }

class AutowareROSBridge : public OSIBridge
{
public:
  explicit AutowareROSBridge(std::string ip_address)
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

    pub_tls_ = nh_.advertise<perception_msgs::TrafficLights>("/grpc/traffic_lights", 1);

    pub_objs_ = nh_.advertise<cyber_perception_msgs::PerceptionObstacles>("/grpc/objects", 1);

    std::cout << "finished" << std::endl;

    TrafficLightIdMathching();
    TrafficLightOSIToKetiMatching();

    is_initialized_ = true;
  }

  virtual ~AutowareROSBridge() = default;

  struct MsgResult
  {
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table; // seq: 디버깅용 -> 빼도 상관없음
    std::queue<std::future<std::pair<size_t,perception_msgs::TrafficLights>>> tl_res_table;
    std::queue<std::future<std::pair<size_t,cyber_perception_msgs::PerceptionObstacles>>> obj_res_table;
  };

  void StartBridge(){
    if(!is_initialized_){
      std::cout << "KETI Converter has Not been initiated!" << std::endl;
      return;
    }

    converting_thread_ = std::thread(&AutowareROSBridge::ConvertThread, this);
    publish_thread_ = std::thread(&AutowareROSBridge::PublishThread, this);
  }

  void Stop(){
    converting_thread_.join();
    publish_thread_.join();
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
            tl_id_match_table_[morai_id].push_back(keti_id);
            // cout << "keti id : " << keti_id << endl;
          }
        }
      }
      // cout << "\n";
    }
    fs.close();
  }

  void TrafficLightOSIToKetiMatching(){
    std::string csv_path, line, temp_string;
    int phase_code;
    std::ifstream fs; 

    csv_path = ros::package::getPath("async_stream") + "/src/matching_table/osi_to_keti_matching_tl.csv";
    fs.open(csv_path, std::ios::in);

    if (!fs.is_open()) {
      std::cout << "can not open file!" << std::endl;
      return;
    }

    while(getline(fs, line)){
      std::stringstream line_str(line);
      for( int i = 0 ; i < 2 ; i++){
        getline(line_str, temp_string, ':');
        if(i == 0){
          phase_code = std::stoi(temp_string);
        }
        if(i == 1){
          switch(Hash(temp_string.c_str())){
            case Hash("OFF"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::OFF;
              break;
            case Hash("GREEN"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::GREEN;
              break;
            case Hash("GREEN_LEFT"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::GREEN_LEFT;
              break;
            case Hash("RED"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::RED;
              break;
            case Hash("RED_LEFT"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::RED_LEFT;
              break;
            case Hash("YELLOW"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::YELLOW;
              break;
            case Hash("RED_YELLOW"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::RED_YELLOW;
              break;
            case Hash("YELLOW_GREEN4"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::YELLOW_GREEN4;
              break;
            case Hash("YELLOW_OTHER"):
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::YELLOW_OTHER;
              break;
            default:
              tl_phase_matching_table_[phase_code].signal_phase = perception_msgs::TrafficSignalPhase::UNKNOWN;
              break;
          }
        }
      }
    }
    fs.close();
    for(auto phase : tl_phase_matching_table_){
      std::cout << "phase code : " << phase.first << ", phase type : " << +phase.second.signal_phase << std::endl;
    }
  }

  static int CalculatePhsaeCode(std::vector<osi3::TrafficLight> osi_tls){
    int res = 0;
    
    for(auto osi_tl : osi_tls){
      switch(osi_tl.classification().color()){
        case osi3::TrafficLight::Classification::COLOR_RED:
          res += osi_tl.classification().mode() * 1000;
          break;
        case osi3::TrafficLight::Classification::COLOR_YELLOW:
          res += osi_tl.classification().mode() * 100;
          break;
        case osi3::TrafficLight::Classification::COLOR_GREEN:
          {
            if(osi_tl.classification().icon() == osi3::TrafficLight::Classification::ICON_ARROW_LEFT){
              res += osi_tl.classification().mode() * 10;  
            }else if(osi_tl.classification().icon() == osi3::TrafficLight::Classification::ICON_NONE){
              res += osi_tl.classification().mode();
            }
            break;
          }
        default:
          break;
      }
    }

    return res;
  }

  static std::pair<size_t,sensor_msgs::PointCloud2> ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                                                              int lidar_id, size_t seq){

    // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    // std::cout << "ProcLidar start lidar : " << lidar_id << ", seq : " << seq << std::endl;

    // if(seq % 2 == 1){
    //   ros::Duration(1.0).sleep();
    // }

    // auto lidar_sensor_view = msg->lidar_sensor_view_;
    // auto lidar_id = msg->lidar_id_;
    // auto seq = msg->seq_;

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "velodyne";
    // cloud_msg.header.frame_id = "hero/lidar";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.width = lidar_sensor_view->view_configuration().directions_size();
    cloud_msg.height = 1;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step*cloud_msg.width*cloud_msg.height;

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    int speed_of_light = 299792458; // m/s
    for ( size_t i = 0; i < lidar_sensor_view->view_configuration().directions_size(); i++ ){
      auto direction = lidar_sensor_view->view_configuration().directions()[i];
      auto reflection = lidar_sensor_view->reflection()[i];

      float magnitude = reflection.time_of_flight()*speed_of_light;
      // std::normal_distribution<> distr(0.0, 0.1);
      // std::random_device rd;
      // std::mt19937 gen(rd());
      // float error = distr(gen); 
      // magnitude += error;

      unsigned char bytes[4];
      float x = magnitude*direction.x();
      memcpy(bytes, &x, sizeof x);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);
      float y = magnitude*direction.y();
      memcpy(bytes, &y, sizeof y);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);        
      float z = magnitude*direction.z();
      memcpy(bytes, &z, sizeof z);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);        
      float intensity = reflection.signal_strength();
      memcpy(bytes, &intensity, sizeof intensity);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);
    }

    // std::chrono::duration<double> running_time = std::chrono::system_clock::now() - now;
    // std::cout.precision(3);
    // std::cout << std::fixed << "lidar converting running time : " << running_time.count() * 1000 << "ms" << std::endl;

    // std::cout << "ProcLidar end lidar : " << lidar_id << ", seq : " << seq << std::endl;
    return std::make_pair(seq,cloud_msg);
  }

  static std::pair<size_t,sensor_msgs::CompressedImage> ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                                   int camera_id, size_t seq){
    std::cout << "ProcCamera start camera : " << camera_id << ", seq : " << seq << std::endl;
    // std::cout << "ProcCamera start camera : " << camera_id << std::endl;

    sensor_msgs::CompressedImage img;
    img.header.stamp = ros::Time::now();
    img.format = "png";
    img.data = {camera_sensor_view->image_data().begin(),
                camera_sensor_view->image_data().end()};

    std::cout << "ProcCamera end camera : " << camera_id << ", seq : " << seq << std::endl;
    // std::cout << "ProcCamera end camera : " << camera_id << std::endl;

    return std::make_pair(seq,img);
  }

  static std::pair<size_t,perception_msgs::TrafficLights> ProcTL(std::shared_ptr<RepeatedPtrField<osi3::TrafficLight>> osi_tls,
                                                                 std::unordered_map<std::string, std::vector<std::string>> *tl_id_match_table,
                                                                 std::unordered_map<int, perception_msgs::TrafficSignalPhase> *tl_phase_matching_table,
                                                                 size_t seq){
    std::cout << "ProcTL start seq : " << seq << std::endl;

    perception_msgs::TrafficLights out_traffic_lights;

    std::cout << "ProcTL end seq : " << seq << std::endl;

    return std::make_pair(seq,out_traffic_lights);
  }

  static std::pair<size_t,cyber_perception_msgs::PerceptionObstacles> ProcObj(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                              std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                              size_t seq){
    std::cout << "ProcObj start, seq : " << seq << std::endl;

    cyber_perception_msgs::PerceptionObstacles obstacles;

    obstacles.header.frame_id = "base_link";

    if(osi_moving_objs.get()->size() == 1 && osi_moving_objs.get()->Get(0).id().value() == 0 &&
       osi_stationary_objs.get()->size() == 1 && osi_stationary_objs.get()->Get(0).id().value() == 0 ){

      return std::make_pair(seq,obstacles);
    }

    for(int i = 0 ; i < osi_moving_objs.get()->size() ; i++){
      auto osi_moving_obj = osi_moving_objs.get()->Get(i);
      cyber_perception_msgs::PerceptionObstacle obs;
      // id
      obs.id = osi_moving_obj.id().value();
      // type
      if(osi_moving_obj.type() == osi3::MovingObject::TYPE_PEDESTRIAN){
        obs.type.type = cyber_perception_msgs::ObstacleType::PEDESTRIAN;
      }
      else if(osi_moving_obj.type() == osi3::MovingObject::TYPE_VEHICLE){
        switch(osi_moving_obj.vehicle_classification().type()){
          case osi3::MovingObject::VehicleClassification::TYPE_BICYCLE:
          case osi3::MovingObject::VehicleClassification::TYPE_MOTORBIKE:
            obs.type.type = cyber_perception_msgs::ObstacleType::BICYCLE;
            break;
          default:
            obs.type.type = cyber_perception_msgs::ObstacleType::VEHICLE;
            break;
        }
      }
      // size (m)
      obs.length = osi_moving_obj.base().dimension().length();
      obs.width = osi_moving_obj.base().dimension().width();
      obs.height = osi_moving_obj.base().dimension().height();
      // heading
      obs.theta = osi_moving_obj.base().orientation().yaw();
      // position
      obs.position.x = osi_moving_obj.base().position().x();
      obs.position.y = osi_moving_obj.base().position().y();
      obs.position.z = osi_moving_obj.base().position().z();
      // velocity
      obs.velocity.x = osi_moving_obj.base().velocity().x();
      obs.velocity.y = osi_moving_obj.base().velocity().y();
      // acceleration
      obs.acceleration.x = osi_moving_obj.base().acceleration().x();
      obs.acceleration.y = osi_moving_obj.base().acceleration().y();

      obstacles.perception_obstacle.push_back(obs);
    }

    for(int i = 0 ; i < osi_stationary_objs.get()->size() ; i++){
      auto osi_stationary_obj = osi_stationary_objs.get()->Get(i);
      cyber_perception_msgs::PerceptionObstacle obs;
      // id
      obs.id = osi_stationary_obj.id().value();
      // type
      obs.type.type = cyber_perception_msgs::ObstacleType::UNKNOWN_UNMOVABLE;
      // size (m)
      obs.length = osi_stationary_obj.base().dimension().length();
      obs.width = osi_stationary_obj.base().dimension().width();
      obs.height = osi_stationary_obj.base().dimension().height();
      // heading
      obs.theta = osi_stationary_obj.base().orientation().yaw();
      // velocity
      obs.velocity.x = 0.0;
      obs.velocity.y = 0.0;
      // acceleration
      obs.acceleration.x = 0.0;
      obs.acceleration.y = 0.0;
      // position
      obs.position.x = osi_stationary_obj.base().position().x();
      obs.position.y = osi_stationary_obj.base().position().y();
      obs.position.z = osi_stationary_obj.base().position().z();

      obstacles.perception_obstacle.push_back(obs);
    }

    std::cout << "ProcObj end seq : " << seq << std::endl;

    return std::make_pair(seq,obstacles);
  }


  void ConvertThread(){ 
    std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table;
    size_t tl_seq = 0, obj_seq = 0;
    
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
          std::cout << "start get sensor view" << std::endl;
          sensor_view = sensor_view_buf_.front();
          sensor_view_buf_.pop();
          std::cout << "finish get sensor view" << std::endl;
      } else continue;

      // std::cout << "lidar_sensor_view size : " << sensor_view.lidar_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
        auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
        int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();
        size_t seq = lidar_seq_table[lidar_id];

        // std::cout << "start Add Conv Thread lidar id " << lidar_id <<  " seq : " << seq << std::endl;
        msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&AutowareROSBridge::ProcLidar, lidar_sensor_view, lidar_id, seq));
        // std::cout << "finish Add Conv Thread lidar id " << lidar_id <<  " seq : " << seq << std::endl;

        lidar_seq_table[lidar_id]++;
      }

      // std::cout << "camera_sensor_view size : " << sensor_view.camera_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
        auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
        int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
        size_t seq = camera_seq_table[camera_id];

        // std::cout << "start Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;
        msg_result_.camera_res_table[camera_id].push(keti::task::Async(&AutowareROSBridge::ProcCamera, camera_sensor_view, camera_id, seq));
        // std::cout << "finish Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;

        camera_seq_table[camera_id]++;
      }

      if ( sensor_view.global_ground_truth().traffic_light_size() > 0 ){
        auto osi_tls = std::make_shared<RepeatedPtrField<osi3::TrafficLight>>(sensor_view.global_ground_truth().traffic_light());

        msg_result_.tl_res_table.push(keti::task::Async(&AutowareROSBridge::ProcTL, osi_tls, &tl_id_match_table_, &tl_phase_matching_table_, tl_seq));
        tl_seq++;
      }

      if ( sensor_view.global_ground_truth().moving_object_size() > 0 || sensor_view.global_ground_truth().stationary_object_size() > 0 ){
        auto moving_objs = std::make_shared<RepeatedPtrField<MovingObject>>(sensor_view.global_ground_truth().moving_object());
        auto stationary_objs = std::make_shared<RepeatedPtrField<StationaryObject>>(sensor_view.global_ground_truth().stationary_object());

        msg_result_.obj_res_table.push(keti::task::Async(&AutowareROSBridge::ProcObj, moving_objs, stationary_objs, obj_seq));
        obj_seq++;
      }

      // std::chrono::duration<double> running_time = std::chrono::system_clock::now() - now;
      // std::cout.precision(3);
      // std::cout << std::fixed << "converting running time : " << running_time.count() * 1000 << "ms" << std::endl;
    }
  }

  void PublishThread(){
    // std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table;
    // for(int i = 0 ; i < num_of_lidar_ ; i++){
    //   lidar_seq_table[i] = 0;  
    // }

    // for(int i = 0 ; i < num_of_camera_ ; i++){
    //   camera_seq_table[i] = 0;  
    // }
    int test = 0;

    while(true){
      if(test < 100){
        std::cout << "start publish thread" << std::endl;
        test++;
      }
      
      // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      for(int i = 0 ; i < num_of_lidar_ ; i++){
        if(msg_result_.lidar_res_table.count(i) == 0){
          continue;
        }

        if(msg_result_.lidar_res_table[i].size() == 0){
          continue;
        }

        std::cout << "try to publish lidar topic : " << i << std::endl;
        auto msg = msg_result_.lidar_res_table[i].front().get();
        std::cout << "check lidar id : " << i << " seq : " << msg.first << std::endl; 
        
        pub_clouds_[i].publish(msg.second);

        std::cout << "publish finished lidar id : " << i << " seq : " << msg.first << std::endl;

        {
          std::cout << "lidar " << i << " erase start" << std::endl;
          msg_result_.lidar_res_table[i].pop();
          std::cout << "lidar " << i << " erase end" << std::endl;
        }

        std::cout << "result size : " << msg_result_.lidar_res_table[i].size() << std::endl;
      }

      for(int i = 0 ; i < num_of_camera_ ; i++){
        if(msg_result_.camera_res_table.count(i) == 0){
          continue;
        }

        if(msg_result_.camera_res_table[i].size() == 0){
          continue;
        }

        std::cout << "try to publish camera topic : " << i << std::endl;
        auto msg = msg_result_.camera_res_table[i].front().get();
        std::cout << "check camera id : " << i << " seq : " << msg.first << std::endl; 
        
        pub_imgs_[i].publish(msg.second);

        std::cout << "publish finished camera id : " << i << " seq : " << msg.first << std::endl;

        {
          std::cout << "camera " << i << " erase start" << std::endl;
          msg_result_.camera_res_table[i].pop();
          std::cout << "camera " << i << " erase end" << std::endl;
        }

        std::cout << "result size : " << msg_result_.camera_res_table[i].size() << std::endl;
      }

      if(msg_result_.tl_res_table.size() != 0){
        std::cout << "try to publish tl topic "<< std::endl;
        auto msg = msg_result_.tl_res_table.front().get();
        std::cout << "check tl seq : " << msg.first << std::endl; 
        
        pub_tls_.publish(msg.second);

        std::cout << "tl publish finished seq : " << msg.first << std::endl;

        {
          std::cout << "tl erase start" << std::endl;
          msg_result_.tl_res_table.pop();
          std::cout << "tl erase end" << std::endl;
        }

        std::cout << "result size : " << msg_result_.tl_res_table.size() << std::endl;

      }

      if(msg_result_.obj_res_table.size() != 0){
        std::cout << "try to publish obj topic "<< std::endl;
        auto msg = msg_result_.obj_res_table.front().get();
        std::cout << "check obj seq : " << msg.first << std::endl; 
        
        pub_objs_.publish(msg.second);

        std::cout << "obj publish finished seq : " << msg.first << std::endl;

        {
          std::cout << "obj erase start" << std::endl;
          msg_result_.obj_res_table.pop();
          std::cout << "obj erase end" << std::endl;
        }

        std::cout << "result size : " << msg_result_.obj_res_table.size() << std::endl;

      }

    }
  }

private:
  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> pub_imgs_, pub_clouds_;
  ros::Publisher pub_tls_, pub_objs_;
  int num_of_camera_, num_of_lidar_;
  ros::Subscriber sub_is_changed_offset_;

  // converting
  MsgResult msg_result_;
  std::unordered_map<std::string, std::vector<std::string>> tl_id_match_table_;
  std::unordered_map<int, perception_msgs::TrafficSignalPhase> tl_phase_matching_table_;

  bool is_initialized_ = false;
};