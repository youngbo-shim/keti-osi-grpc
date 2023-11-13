#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>

#include "sensorview_rpc.grpc.pb.h"
#include "task/task.h"
#include "osi_bridge.cc"
#include "keti_ros_converter.h"

using google::protobuf::RepeatedPtrField;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;

using namespace keti::hdmap;

class KetiROSBridge : public OSIBridge
{
public:
  explicit KetiROSBridge(std::string ip_address)
          : OSIBridge(ip_address)
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

    is_initialized_ = true;
  }

  virtual ~KetiROSBridge() = default;

  struct MsgResult
  {
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table; // seq: 디버깅용 -> 빼도 상관없음
    std::queue<std::future<std::pair<size_t,perception_msgs::TrafficLights>>> tl_res_table;
    std::queue<std::future<std::pair<size_t,cyber_perception_msgs::PerceptionObstacles>>> obj_res_table;
  };

  void StartBridge(){
    if(!is_initialized_){
      std::cout << "KETI ROS Bridge has Not been initiated!" << std::endl;
      return;
    }

    converting_thread_ = std::thread(&KetiROSBridge::ConvertThread, this);
    publish_thread_ = std::thread(&KetiROSBridge::PublishThread, this);
    ros::spin();
  }

  void Stop(){
    converting_thread_.join();
    publish_thread_.join();
  }

  void CallbackUpdateOffsetParams(const std_msgs::Bool& check){

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
        msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&KetiROSConverter::ProcLidar, &converter_, lidar_sensor_view, lidar_id, seq));
        // std::cout << "finish Add Conv Thread lidar id " << lidar_id <<  " seq : " << seq << std::endl;

        lidar_seq_table[lidar_id]++;
      }

      // std::cout << "camera_sensor_view size : " << sensor_view.camera_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
        auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
        int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
        size_t seq = camera_seq_table[camera_id];

        // std::cout << "start Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;
        msg_result_.camera_res_table[camera_id].push(keti::task::Async(&KetiROSConverter::ProcCamera, &converter_, camera_sensor_view, camera_id, seq));
        // std::cout << "finish Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;

        camera_seq_table[camera_id]++;
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

        std::cout << "lidar " << i << " erase start" << std::endl;
        msg_result_.lidar_res_table[i].pop();
        std::cout << "lidar " << i << " erase end" << std::endl;

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

        std::cout << "camera " << i << " erase start" << std::endl;
        msg_result_.camera_res_table[i].pop();
        std::cout << "camera " << i << " erase end" << std::endl;

        std::cout << "result size : " << msg_result_.camera_res_table[i].size() << std::endl;
      }

      if(msg_result_.tl_res_table.size() != 0){
        std::cout << "try to publish tl topic "<< std::endl;
        auto msg = msg_result_.tl_res_table.front().get();
        std::cout << "check tl seq : " << msg.first << std::endl; 
        
        pub_tls_.publish(msg.second);

        std::cout << "tl publish finished seq : " << msg.first << std::endl;

        std::cout << "tl erase start" << std::endl;
        msg_result_.tl_res_table.pop();
        std::cout << "tl erase end" << std::endl;

        std::cout << "result size : " << msg_result_.tl_res_table.size() << std::endl;

      }

      if(msg_result_.obj_res_table.size() != 0){
        std::cout << "try to publish obj topic "<< std::endl;
        auto msg = msg_result_.obj_res_table.front().get();
        std::cout << "check obj seq : " << msg.first << std::endl; 
        
        pub_objs_.publish(msg.second);

        std::cout << "obj publish finished seq : " << msg.first << std::endl;

        std::cout << "obj erase start" << std::endl;
        msg_result_.obj_res_table.pop();
        std::cout << "obj erase end" << std::endl;

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

  // hdmap
  std::shared_ptr<HDMap> hdmap_;

  // converting
  KetiROSConverter converter_;
  MsgResult msg_result_;

  bool is_initialized_ = false;
};