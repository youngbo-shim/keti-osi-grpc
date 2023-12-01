#include "keti_ros_converter.h"
#include "utils/string_utils.h"

KetiROSConverter::KetiROSConverter()
{
  TrafficLightIdMathching();
  TrafficLightOSIToKetiMatching();
}

void KetiROSConverter::TrafficLightIdMathching(){
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

void KetiROSConverter::TrafficLightOSIToKetiMatching(){
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
  // for(auto phase : tl_phase_matching_table_){
  //   std::cout << "phase code : " << phase.first << ", phase type : " << +phase.second.signal_phase << std::endl;
  // }
}

std::pair<size_t,sensor_msgs::PointCloud2> KetiROSConverter::ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                                                                        int lidar_id, size_t seq){
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "lidar" + std::to_string(lidar_id);
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
  cloud_msg.fields[0].count = 1;
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;
  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;
  cloud_msg.fields[3].name = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;

  cloud_msg.is_dense = true;

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
  return std::make_pair(seq,cloud_msg);
}

std::pair<size_t,sensor_msgs::CompressedImage> KetiROSConverter::ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                                            int camera_id, size_t seq){
  sensor_msgs::CompressedImage img;
  img.header.stamp = ros::Time::now();
  img.format = "png";
  img.data = {camera_sensor_view->image_data().begin(),
              camera_sensor_view->image_data().end()};

  return std::make_pair(seq,img);
}

  // void RadarOSIToGeneral(RadarSensorView& radar_osi,
  //                       std::vector<radar_msgs::RadarReturn>* radarreturns){
  //   float radar_frequency; // Need to know Radar frequency
  //   for (auto reflection : radar_osi.reflection()){
  //     radar_msgs::RadarReturn radar_obj;    

  //     radar_obj.range = reflection.time_of_flight() * SPEED_OF_LIGHT;
  //     radar_obj.azimuth = reflection.source_horizontal_angle();
  //     radar_obj.elevation = reflection.source_vertical_angle();
  //     radar_obj.doppler_velocity = reflection.doppler_shift() * SPEED_OF_LIGHT / radar_frequency;
  //     radar_obj.amplitude = reflection.signal_strength();

  //     radarreturns->push_back(radar_obj);
  //   }
  // }

std::pair<size_t, radar_msgs::RadarScan> KetiROSConverter::ProcRadar(std::shared_ptr<RadarSensorView> radar_sensor_view,
                                                    int radar_id, size_t seq){
  radar_msgs::RadarScan msg;
  float radar_frequency = 20.0;  // Need to get as param later

  msg.header.stamp = ros::Time::now();
  for (auto reflection : radar_sensor_view->reflection()){
    radar_msgs::RadarReturn radar_obj;
    
    radar_obj.range = reflection.time_of_flight() * SPEED_OF_LIGHT;
    radar_obj.azimuth = reflection.source_horizontal_angle();
    radar_obj.elevation = reflection.source_vertical_angle();
    radar_obj.doppler_velocity = reflection.doppler_shift() * SPEED_OF_LIGHT / radar_frequency;
    radar_obj.amplitude = reflection.signal_strength();

    msg.returns.push_back(radar_obj);
  }

  return std::make_pair(seq,msg);
}

int KetiROSConverter::CalculatePhsaeCode(std::vector<osi3::TrafficLight> osi_tls){
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

std::pair<size_t,perception_msgs::TrafficLights> KetiROSConverter::ProcTL(std::shared_ptr<RepeatedPtrField<osi3::TrafficLight>> osi_tls,
                                                                          std::shared_ptr<HDMap> hdmap,
                                                                          size_t seq){
  perception_msgs::TrafficLights out_traffic_lights;
  perception_msgs::TrafficLight traffic_light;
  std::unordered_map<std::string, std::vector<osi3::TrafficLight>> osi_tl_container;

  out_traffic_lights.header.frame_id = "map";
  out_traffic_lights.header.stamp = ros::Time::now();

  for(int i = 0 ; i < osi_tls.get()->size() ; i++){
    auto osi_tl = osi_tls.get()->Get(i);
    std::string morai_id = osi_tl.source_reference(0).identifier(0);

    if(tl_id_match_table_.count(morai_id) == 0){
      // std::cout << "id matching failed, need to update id_table id : " << morai_id << std::endl;
      continue;
    }

    osi_tl_container[morai_id].push_back(osi_tl);
  }

  for(auto tls : osi_tl_container){
    auto tl_ptr = hdmap->GetTrafficLightById(tl_id_match_table_.at(tls.first)[0]);
    if(!tl_ptr){
      // std::cout << "GetTrafficLightById Failed, id : " << tls.first << std::endl;
      continue;
    }
    traffic_light.id = tl_ptr->id();
    traffic_light.signal_group_id = tl_ptr->link_id();
    traffic_light.type = tl_ptr->type();
    traffic_light.signal_phase = tl_phase_matching_table_.at(CalculatePhsaeCode(tls.second));
    traffic_light.point.x = tl_ptr->point().x();
    traffic_light.point.y = tl_ptr->point().y();
    traffic_light.point.z = tl_ptr->point().z();
    traffic_light.heading = tl_ptr->heading();

    // std::cout << "morai id : " << tls.first << ", keti id : " << tl_ptr->id()
    //           << ", type : " << +traffic_light.type << ", signal_group_id : " << traffic_light.signal_group_id
    //           << ", phase : " << +traffic_light.signal_phase.signal_phase 
    //           << ", blink : " << +traffic_light.signal_phase.blink << " Keti Ros Converter" << std::endl;
    
    out_traffic_lights.traffic_lights.push_back(traffic_light);

    for(int i = 1 ; i < tl_id_match_table_.at(tls.first).size() ; i++){
      tl_ptr = hdmap->GetTrafficLightById(tl_id_match_table_.at(tls.first)[i]);
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

  return std::make_pair(seq,out_traffic_lights);
}

std::pair<size_t,cyber_perception_msgs::PerceptionObstacles> KetiROSConverter::ProcObj(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                                       std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                                       size_t seq){
  cyber_perception_msgs::PerceptionObstacles obstacles;

  obstacles.header.frame_id = "base_link";
  obstacles.header.timestamp_sec = ros::Time::now().toSec();

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

  return std::make_pair(seq,obstacles);
}

std::tuple<size_t,control_msgs::VehicleState,geometry_msgs::PoseStamped,geometry_msgs::TransformStamped> KetiROSConverter::ProcEgoVehicleState(std::shared_ptr<HostVehicleData> host_vehicle_data,
                                                                                                                                               size_t seq, std::shared_ptr<HDMap> hdmap){
    control_msgs::VehicleState out_vehicle_state;
    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::TransformStamped odom_trans;

    out_vehicle_state.header.stamp = ros::Time::now();
    out_vehicle_state.autonomous_status = 2;
    out_vehicle_state.x_utm = host_vehicle_data->vehicle_motion().position().x();
    out_vehicle_state.y_utm = host_vehicle_data->vehicle_motion().position().y();
    out_vehicle_state.v_ego = math::ms2kmh(host_vehicle_data->vehicle_motion().velocity().x());
    out_vehicle_state.a_x = host_vehicle_data->vehicle_motion().acceleration().x();
    out_vehicle_state.steer_angle = math::rad2deg(host_vehicle_data->vehicle_steering().vehicle_steering_wheel().angle());
    out_vehicle_state.heading_utm = host_vehicle_data->vehicle_motion().orientation().yaw();
    out_vehicle_state.yaw_rate = host_vehicle_data->vehicle_motion().orientation_rate().yaw();

    cur_pose.header.frame_id = "map";
    cur_pose.header.stamp = ros::Time::now();
    cur_pose.pose.position.x = out_vehicle_state.x_utm;  
    cur_pose.pose.position.y = out_vehicle_state.y_utm;  
    cur_pose.pose.orientation = tf::createQuaternionMsgFromYaw(out_vehicle_state.heading_utm);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = cur_pose.pose.position.x-hdmap->x_offset();
    odom_trans.transform.translation.y = cur_pose.pose.position.y-hdmap->y_offset();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = cur_pose.pose.orientation;

    return {seq,out_vehicle_state,cur_pose,odom_trans};
  }