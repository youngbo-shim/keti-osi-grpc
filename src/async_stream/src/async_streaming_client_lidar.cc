#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/support/async_stream.h>

// Sensor Error(Gausian) Modeling
#include <random>

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

using grpc::Channel;
using grpc::ClientAsyncReader;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using google::protobuf::RepeatedPtrField;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;
using osi3::MovingObject;
using osi3::StationaryObject;

using namespace keti::hdmap;

static constexpr unsigned int Hash(const char* str){
  return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * Hash(str + 1) : 8603;
}

class SensorViewClient
{
public:
  SensorViewClient(std::shared_ptr<Channel> channel)
      : stub_(SensorViewRPC::NewStub(channel)), state_(CallState::NewCall)
  {
    // set hdmap -> move on to converting class later
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

    sub_is_changed_offset_ = nh_.subscribe("/is_changed_offset",1, &SensorViewClient::CallbackUpdateOffsetParams, this);

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

    this->onNext(true);
  }

  ~SensorViewClient()
  {
    void *ignoredTag = nullptr;
    bool ok = false;
    while (cq_.Next(&ignoredTag, &ok));

    call_->reader->Finish(&call_->status, call_);
  }

  enum class CallState {
    NewCall,
    SendingRequest,
    ReceivingData,
    CallComplete
  };

  struct MsgResult
  {
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::PointCloud2>>>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table; // seq: 디버깅용 -> 빼도 상관없음
    std::queue<std::future<std::pair<size_t,perception_msgs::TrafficLights>>> tl_res_table;
    std::queue<std::future<std::pair<size_t,cyber_perception_msgs::PerceptionObstacles>>> obj_res_table;
  };

  // move on to converting class later
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

  void handleNewCallState() {
    call_ = new AsyncClientCall;
    call_->request.set_host_name("localhost:50051");

    call_->reader = stub_->PrepareAsyncGetSensorView(&call_->context, call_->request, &cq_);

    // StartCall initiates the RPC call
    state_ = CallState::SendingRequest;
    call_->reader->StartCall(call_);
  }

  void handleSendingRequestState() {
    state_ = CallState::ReceivingData;

    call_->reader->Read(&call_->reply, call_);    
  }

  void handleReceivingDataState() {
    static size_t num_data = 0;
    const auto &sensor_view = call_->reply;

    std::chrono::time_point<std::chrono::system_clock> now =
          std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto current_nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;

    double sensor_view_stamp = sensor_view.timestamp().seconds()+sensor_view.timestamp().nanos()*1e-9;
    double communication_time = current_nanoseconds - sensor_view_stamp;
    // std::cout.precision(3);
    // std::cout << std::fixed << "[sensor, current] : " << sensor_view_stamp << ", " << current_nanoseconds << std::endl;
    // std::cout << "[msg size, communication speed] : " << sensor_view.ByteSizeLong() << ", " << (8.*sensor_view.ByteSizeLong()/(1024*1024))/communication_time << "Mbps" << std::endl;
    
    {
      std::lock_guard<std::mutex> lock(sensor_view_mutex_);
      sensor_view_buf_.push(sensor_view);
    }

    call_->reply.Clear();
    call_->reader->Read(&call_->reply, call_);

  }

  void handleCallCompleteState()
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

  bool onNext(bool ok)
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

  void AsyncCompleteRpc()
  {
    void *got_tag;
    bool ok = false;

    // Block until the next result is available in the completion queue "cq".
    while (1)
    {
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      cq_.Next(&got_tag, &ok);

      auto duration = now.time_since_epoch();
      auto micros =
          std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
      // std::cout << "[" << micros << "], Next!!" << std::endl;
      AsyncClientCall *call = static_cast<AsyncClientCall *>(got_tag);      
      if(!ok) continue;
      if (!this->onNext(ok))
      {
        break;
      }
    }
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
                                                                 std::shared_ptr<HDMap> hdmap,
                                                                 std::unordered_map<std::string, std::vector<std::string>> *tl_id_match_table,
                                                                 std::unordered_map<int, perception_msgs::TrafficSignalPhase> *tl_phase_matching_table,
                                                                 size_t seq){
    std::cout << "ProcTL start seq : " << seq << std::endl;

    perception_msgs::TrafficLights out_traffic_lights;
    perception_msgs::TrafficLight traffic_light;
    std::unordered_map<std::string, std::vector<osi3::TrafficLight>> osi_tl_container;

    for(int i = 0 ; i < osi_tls.get()->size() ; i++){
      auto osi_tl = osi_tls.get()->Get(i);
      std::string morai_id = osi_tl.source_reference(0).identifier(0);

      if(tl_id_match_table->count(morai_id) == 0){
        std::cout << "id matching failed, need to update id_table id : " << morai_id << std::endl;
        continue;
      }

      osi_tl_container[morai_id].push_back(osi_tl);
    }

    for(auto tls : osi_tl_container){
      auto tl_ptr = hdmap->GetTrafficLightById(tl_id_match_table->at(tls.first)[0]);
      if(!tl_ptr){
        std::cout << "GetTrafficLightById Failed, id : " << tls.first << std::endl;
        continue;
      }
      traffic_light.id = tl_ptr->id();
      traffic_light.signal_group_id = tl_ptr->link_id();
      traffic_light.type = tl_ptr->type();
      traffic_light.signal_phase = tl_phase_matching_table->at(CalculatePhsaeCode(tls.second));
      traffic_light.point.x = tl_ptr->point().x();
      traffic_light.point.y = tl_ptr->point().y();
      traffic_light.point.z = tl_ptr->point().z();
      traffic_light.heading = tl_ptr->heading();

      std::cout << "morai id : " << tls.first << ", keti id : " << tl_ptr->id()
                << ", type : " << +traffic_light.type << ", signal_group_id : " << traffic_light.signal_group_id
                << ", phase : " << +traffic_light.signal_phase.signal_phase 
                << ", blink : " << +traffic_light.signal_phase.blink << std::endl;
      
      out_traffic_lights.traffic_lights.push_back(traffic_light);

      for(int i = 1 ; i < tl_id_match_table->at(tls.first).size() ; i++){
        tl_ptr = hdmap->GetTrafficLightById(tl_id_match_table->at(tls.first)[i]);
        traffic_light.id = tl_ptr->id();
        traffic_light.point.x = tl_ptr->point().x();
        traffic_light.point.y = tl_ptr->point().y();
        traffic_light.point.z = tl_ptr->point().z();
        traffic_light.heading = tl_ptr->heading();
        // std::cout << "morai id : " << osi_tls.first << ", keti id : " << tl_ptr->id()
        //           << ", type : " << +traffic_light.type << ", signal_group_id : " << traffic_light.signal_group_id
        //           << ", phase : " << +traffic_light.signal_phase.signal_phase 
        //           << ", blink : " << +traffic_light.signal_phase.blink << std::endl;
        out_traffic_lights.traffic_lights.push_back(traffic_light);
      }
    }

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
          sensor_view = sensor_view_buf_.front();
          sensor_view_buf_.pop();
      } else continue;

      // std::cout << "lidar_sensor_view size : " << sensor_view.lidar_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
        auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
        int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();
        size_t seq = lidar_seq_table[lidar_id];

        // std::cout << "start Add Conv Thread lidar id " << lidar_id <<  " seq : " << seq << std::endl;
        msg_result_.lidar_res_table[lidar_id].push(keti::task::Async(&SensorViewClient::ProcLidar, lidar_sensor_view, lidar_id, seq));
        // std::cout << "finish Add Conv Thread lidar id " << lidar_id <<  " seq : " << seq << std::endl;

        lidar_seq_table[lidar_id]++;
      }

      // std::cout << "camera_sensor_view size : " << sensor_view.camera_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
        auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
        int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
        size_t seq = camera_seq_table[camera_id];

        // std::cout << "start Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;
        msg_result_.camera_res_table[camera_id].push(keti::task::Async(&SensorViewClient::ProcCamera, camera_sensor_view, camera_id, seq));
        // std::cout << "finish Add Conv Thread camera id " << camera_id <<  " seq : " << seq << std::endl;

        camera_seq_table[camera_id]++;
      }

      if ( sensor_view.global_ground_truth().traffic_light_size() > 0 ){
        auto osi_tls = std::make_shared<RepeatedPtrField<osi3::TrafficLight>>(sensor_view.global_ground_truth().traffic_light());

        msg_result_.tl_res_table.push(keti::task::Async(&SensorViewClient::ProcTL, osi_tls, hdmap_, &tl_id_match_table_, &tl_phase_matching_table_, tl_seq));
        tl_seq++;
      }

      if ( sensor_view.global_ground_truth().moving_object_size() > 0 || sensor_view.global_ground_truth().stationary_object_size() > 0 ){
        auto moving_objs = std::make_shared<RepeatedPtrField<MovingObject>>(sensor_view.global_ground_truth().moving_object());
        auto stationary_objs = std::make_shared<RepeatedPtrField<StationaryObject>>(sensor_view.global_ground_truth().stationary_object());

        msg_result_.obj_res_table.push(keti::task::Async(&SensorViewClient::ProcObj, moving_objs, stationary_objs, obj_seq));
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
  struct AsyncClientCall
  {
    Request request;

    // Container for the data we expect from the server.
    SensorView reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // Storage for the status of the RPC upon completion.
    Status status;

    // std::unique_ptr<ClientAsyncReader<SensorView>> reader;
    std::unique_ptr<ClientAsyncReader<SensorView>> reader;
  };

private:
  AsyncClientCall *call_;
  std::unique_ptr<SensorViewRPC::Stub> stub_;
  CompletionQueue cq_;
  CallState state_;
  std::queue<SensorView> sensor_view_buf_;
  std::mutex sensor_view_mutex_;

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
  std::shared_ptr<HDMap> hdmap_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "grpc_client");

  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
  SensorViewClient client(
      grpc::CreateCustomChannel("localhost:50051",
                                grpc::InsecureChannelCredentials(),
                                args));

  ros::Duration(0.1).sleep();

  std::thread thread_ = std::thread(&SensorViewClient::AsyncCompleteRpc, &client);
  std::thread converting_thread_ = std::thread(&SensorViewClient::ConvertThread, &client);
  std::thread publish_thread_ = std::thread(&SensorViewClient::PublishThread, &client);

  ros::spin();

  thread_.join();
  converting_thread_.join();
  publish_thread_.join();

  return 0;
}