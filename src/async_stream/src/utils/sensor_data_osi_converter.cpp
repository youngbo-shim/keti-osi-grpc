#include "sensor_data_osi_converter.h"

#define STEER_RATIO 12.3

SensorDataOSIConverter::SensorDataOSIConverter(){}
SensorDataOSIConverter::~SensorDataOSIConverter(){}

void SensorDataOSIConverter::CameraSensorToOSI(const sensor_msgs::CompressedImageConstPtr& img_ros, CameraSensorView* camera_osi,
                                               const MountingPosition& camera_mount_pose, const size_t& sensor_id){

  auto camera_view_configuration = camera_osi->mutable_view_configuration();
  camera_osi->mutable_image_data()->resize(img_ros->data.size());
  *camera_osi->mutable_image_data() = {img_ros->data.begin(), img_ros->data.end()};
  camera_view_configuration->mutable_sensor_id()->set_value(sensor_id);
  camera_view_configuration->mutable_mounting_position()->mutable_position()->set_x(camera_mount_pose.position().x());
  camera_view_configuration->mutable_mounting_position()->mutable_position()->set_y(camera_mount_pose.position().y());
  camera_view_configuration->mutable_mounting_position()->mutable_position()->set_z(camera_mount_pose.position().z());
  camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(camera_mount_pose.orientation().roll());
  camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(camera_mount_pose.orientation().pitch());
  camera_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(camera_mount_pose.orientation().yaw());

}

void SensorDataOSIConverter::LidarSensorToOSI(const sensor_msgs::PointCloud2ConstPtr& lidar_ros, LidarSensorView* lidar_osi, 
                          const MountingPosition& lidar_mount_pose, const size_t& num_of_rays, const size_t& sensor_id){

  auto lidar_view_configuration = lidar_osi->mutable_view_configuration();
  lidar_view_configuration->set_number_of_rays_vertical(num_of_rays);
  lidar_view_configuration->set_number_of_rays_horizontal(lidar_ros->width/num_of_rays);
  lidar_view_configuration->set_num_of_pixels(lidar_ros->width*lidar_ros->height);
  lidar_view_configuration->mutable_sensor_id()->set_value(sensor_id);
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_x(lidar_mount_pose.position().x());
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_y(lidar_mount_pose.position().y());
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_z(lidar_mount_pose.position().z());
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(lidar_mount_pose.orientation().roll());
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(lidar_mount_pose.orientation().pitch());
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(lidar_mount_pose.orientation().yaw());

  for (size_t i = 0; i < lidar_ros->data.size();){
    // x
    size_t start_idx = i + lidar_ros->fields[0].offset;
    std::vector<uint8_t> point_buf = 
              std::vector<uint8_t>(lidar_ros->data.begin() + start_idx,
                                  lidar_ros->data.begin() + start_idx + 4);

    float x_value = 0.0;
    assert(point_buf.size() == sizeof(float));
    memcpy(&x_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));

    // y
    start_idx = i + lidar_ros->fields[1].offset;
    point_buf = std::vector<uint8_t>(lidar_ros->data.begin() + start_idx,
                                    lidar_ros->data.begin() + start_idx + 4);

    float y_value = 0.0;
    assert(point_buf.size() == sizeof(float));
    memcpy(&y_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));

    // z
    start_idx = i + lidar_ros->fields[2].offset;
    point_buf = std::vector<uint8_t>(lidar_ros->data.begin() + start_idx,
                                    lidar_ros->data.begin() + start_idx + 4);

    float z_value = 0.0;
    assert(point_buf.size() == sizeof(float));
    memcpy(&z_value, &point_buf[0], std::min(point_buf.size(), sizeof(float)));
    
    // intensity
    start_idx = i + lidar_ros->fields[3].offset;
    point_buf = std::vector<uint8_t>(lidar_ros->data.begin() + start_idx,
                                    lidar_ros->data.begin() + start_idx + 4);

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
    auto reflection = lidar_osi->add_reflection();
    reflection->set_time_of_flight(magnitude_direction/speed_of_light);
    reflection->set_signal_strength(intensity_value);

    i += lidar_ros->fields.back().offset + 4; // 4 is the size of float
  }
}

// Radar
void SensorDataOSIConverter::RadarSensorToOSI(morai_msgs::RadarDetections& radardetections,
                                              RadarSensorView* radar_osi,
                                              const MountingPosition& radar_mount_pose,
                                              const size_t& sensor_id){
  // 1. RadarSensorViewConfiguration
  auto radar_view_configuration = radar_osi->mutable_view_configuration();
  
  radar_view_configuration->mutable_sensor_id()->set_value(sensor_id);
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_x(radar_mount_pose.position().x());
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_y(radar_mount_pose.position().y());  
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_z(radar_mount_pose.position().z());
  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(radar_mount_pose.orientation().roll());
  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(radar_mount_pose.orientation().pitch());
  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(radar_mount_pose.orientation().yaw());

  // 2. Reflection
  float x_l, y_l, z_l, time_of_flight;
  for (auto radar_obj : radardetections.detections){
    auto reflection = radar_osi->add_reflection();

    reflection->set_signal_strength(radar_obj.amplitude);

    time_of_flight = 0.0;
    x_l = std::abs(radar_obj.position.x);
    y_l = std::abs(radar_obj.position.y);
    z_l = std::abs(radar_obj.position.z);

    time_of_flight = std::sqrt(x_l*x_l + y_l*y_l + z_l*z_l) / SPEED_OF_LIGHT;
    reflection->set_time_of_flight(time_of_flight);
    reflection->set_doppler_shift(0);
    reflection->set_source_horizontal_angle(radar_obj.azimuth*M_PI / 180);

    auto elevation_rad = atan2(radar_mount_pose.position().z(), radar_mount_pose.position().y());
    reflection->set_source_vertical_angle(elevation_rad);
  }
}

void SensorDataOSIConverter::RadarSensorToOSI(radar_msgs::RadarScan& radarscans,
                                              RadarSensorView* radar_osi,
                                              const geometry_msgs::PoseStamped::ConstPtr& radar_mount_pose,
                                              const size_t& sensor_id){

  // 1. RadarSensorViewConfiguration
  auto radar_view_configuration = radar_osi->mutable_view_configuration();
  
  radar_view_configuration->mutable_sensor_id()->set_value(sensor_id);
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_x(radar_mount_pose->pose.position.x);
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_y(radar_mount_pose->pose.position.y);  
  radar_view_configuration->mutable_mounting_position()->mutable_position()->set_z(radar_mount_pose->pose.position.z);

  tf::Quaternion q(
    radar_mount_pose->pose.orientation.x,
    radar_mount_pose->pose.orientation.y,
    radar_mount_pose->pose.orientation.z,
    radar_mount_pose->pose.orientation.w   
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(roll);
  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(pitch);
  radar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(yaw);

  // 2. Reflection
  float time_of_flight, doppler_shift, radar_frequency;
  for (auto radar_obj : radarscans.returns){
    auto reflection = radar_osi->add_reflection();

    reflection->set_signal_strength(radar_obj.amplitude);

    time_of_flight = radar_obj.range / SPEED_OF_LIGHT;
    reflection->set_time_of_flight(time_of_flight);

    doppler_shift = radar_obj.doppler_velocity * radar_frequency / SPEED_OF_LIGHT;
    reflection->set_doppler_shift(doppler_shift);

    reflection->set_source_horizontal_angle(radar_obj.azimuth);
    reflection->set_source_vertical_angle(radar_obj.elevation);
  }
}

// void SensorDataOSIConverter::TrafficLightsToOSI(const morai_msgs::GetTrafficLightStatusConstPtr& traffic_light_ros, GroundTruth* traffic_ligth_osi, const ){
void SensorDataOSIConverter::TrafficLightsToOSI(const morai_msgs::GetTrafficLightStatusConstPtr& traffic_light_ros, osi3::TrafficLight* traffic_ligth_gt, osi3::TrafficLight& traffic_ligth_osi) {
  traffic_ligth_gt->mutable_id()->set_value(traffic_ligth_osi.id().value());
  traffic_ligth_gt->mutable_classification()->set_color(traffic_ligth_osi.classification().color());
  traffic_ligth_gt->mutable_classification()->set_icon(traffic_ligth_osi.classification().icon());
  traffic_ligth_gt->mutable_classification()->set_mode(traffic_ligth_osi.classification().mode());
  traffic_ligth_gt->add_source_reference()->add_identifier(traffic_light_ros->trafficLightIndex);
}

void SensorDataOSIConverter::TrafficLightIdMathching(){
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

void SensorDataOSIConverter::TrafficLightMoraiToOSIMatching(){
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

void SensorDataOSIConverter::ObjectMoraiToOSIMatching(){
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

void SensorDataOSIConverter::EgoVehicleStateToOSI(const morai_msgs::EgoVehicleStatusConstPtr& ego_vehicle_state_ros, const sensor_msgs::ImuConstPtr& imu_ros, 
                          const autoware_msgs::VehicleStatusConstPtr& autoware_vehicle_state_ros, HostVehicleData* host_vehicle_osi){
  
  // Position
  double x_utm, y_utm;
  this->GetGeoCoordConv().Conv(ego_vehicle_state_ros->position.x + this->GetMoraiTMOffset()[0],
                               ego_vehicle_state_ros->position.y + this->GetMoraiTMOffset()[1],
                               x_utm, y_utm);

  host_vehicle_osi->mutable_vehicle_motion()->mutable_position()->set_x(x_utm);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_position()->set_y(y_utm);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_position()->set_z(ego_vehicle_state_ros->position.z);

  // Orientation, angular velocity, acceleration
  tf::Quaternion q(imu_ros->orientation.x, imu_ros->orientation.y,
                  imu_ros->orientation.z, imu_ros->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation()->set_roll(roll);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation()->set_pitch(pitch);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation()->set_yaw(yaw);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation_rate()->set_roll(imu_ros->angular_velocity.x);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation_rate()->set_pitch(imu_ros->angular_velocity.y);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_orientation_rate()->set_yaw(imu_ros->angular_velocity.z);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_acceleration()->set_x(imu_ros->linear_acceleration.x);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_acceleration()->set_y(imu_ros->linear_acceleration.y);
  host_vehicle_osi->mutable_vehicle_motion()->mutable_acceleration()->set_z(imu_ros->linear_acceleration.z);
  host_vehicle_osi->mutable_vehicle_steering()->mutable_vehicle_steering_wheel()->set_angle(math::deg2rad(-ego_vehicle_state_ros->wheel_angle*STEER_RATIO));

  // veloicty
  if(autoware_vehicle_state_ros)
    host_vehicle_osi->mutable_vehicle_motion()->mutable_velocity()->set_x(autoware_vehicle_state_ros->speed);
  else{
    host_vehicle_osi->mutable_vehicle_motion()->mutable_velocity()->set_x(ego_vehicle_state_ros->velocity.x);
    host_vehicle_osi->mutable_vehicle_motion()->mutable_velocity()->set_y(ego_vehicle_state_ros->velocity.y);
    host_vehicle_osi->mutable_vehicle_motion()->mutable_velocity()->set_z(ego_vehicle_state_ros->velocity.z);
  }

}
void SensorDataOSIConverter::EmptyObstacleToOSI(osi3::MovingObject* moving_obstacle_osi,
                                                osi3::StationaryObject* stationary_obstacle_osi){
  moving_obstacle_osi->mutable_id()->set_value(0);
  stationary_obstacle_osi->mutable_id()->set_value(0);
}

void SensorDataOSIConverter::NPCObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, 
                                              osi3::MovingObject* moving_obstacle_osi){

  double x_utm, y_utm, x_vel, y_vel, x_accel, y_accel;
  double ego_heading = this->GetEgoVehicleHeading();
  double morai_tm_x_offset = this->GetMoraiTMOffset()[0];
  double morai_tm_y_offset = this->GetMoraiTMOffset()[1];

  moving_obstacle_osi->mutable_id()->set_value(obstacle_ros.unique_id);
  moving_obstacle_osi->set_type(osi3::MovingObject::TYPE_VEHICLE);
  moving_obstacle_osi->mutable_vehicle_classification()->set_type(osi3::MovingObject::VehicleClassification::TYPE_MEDIUM_CAR);

  // vehicle_type
  for(auto search_obj : this->GetMoraiToOSIObstacleMatchingTable()){
    if(search_obj.second.type() != osi3::MovingObject::VehicleClassification::TYPE_BUS &&
      search_obj.second.type() != osi3::MovingObject::VehicleClassification::TYPE_HEAVY_TRUCK){
      continue;
    }
    if(obstacle_ros.name.find(search_obj.first.second) != std::string::npos){
      moving_obstacle_osi->mutable_vehicle_classification()->set_type(search_obj.second.type());
      break;
    }
  }
  // name
  moving_obstacle_osi->add_source_reference()->add_identifier(obstacle_ros.name);
  // size (m)
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_length(obstacle_ros.size.x);
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_width(obstacle_ros.size.y);
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_height(obstacle_ros.size.z);

  // position
  this->GetGeoCoordConv().Conv(obstacle_ros.position.x + morai_tm_x_offset,
                obstacle_ros.position.y + morai_tm_y_offset,
                x_utm, y_utm);
  moving_obstacle_osi->mutable_base()->mutable_position()->set_x(x_utm);
  moving_obstacle_osi->mutable_base()->mutable_position()->set_y(y_utm);
  // moving_obstacle_osi->mutable_base()->mutable_position()->set_z(obstacle_ros.position.z);

  //heading (degree to rad)
  double obj_heading = math::deg2rad(obstacle_ros.heading);
  moving_obstacle_osi->mutable_base()->mutable_orientation()->set_yaw(obj_heading);

  // To Ego vehicle coordinate
  // velocity
  double local_heading = obj_heading - ego_heading;

  double velocity_2d[2] = {math::kmh2ms(obstacle_ros.velocity.x), math::kmh2ms(obstacle_ros.velocity.y)};
  x_vel = cos(local_heading)*velocity_2d[0]
        - sin(local_heading)*velocity_2d[1];
  y_vel = sin(local_heading)*velocity_2d[0]
        + cos(local_heading)*velocity_2d[1];

  // acceleration
  double accel_2d[2] = {obstacle_ros.acceleration.x, obstacle_ros.acceleration.y};    
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

  moving_obstacle_osi->mutable_base()->mutable_velocity()->set_x(x_vel);
  moving_obstacle_osi->mutable_base()->mutable_velocity()->set_y(y_vel);
                                  

  double tmp_x_accel = x_accel;
  double tmp_y_accel = y_accel;

  x_accel = cos(ego_heading)*tmp_x_accel
            -sin(ego_heading)*tmp_y_accel;

  y_accel = sin(ego_heading)*tmp_x_accel
            +cos(ego_heading)*tmp_y_accel;

  moving_obstacle_osi->mutable_base()->mutable_acceleration()->set_x(x_accel);
  moving_obstacle_osi->mutable_base()->mutable_acceleration()->set_y(y_accel);


}
void SensorDataOSIConverter::PedObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, 
                                              osi3::MovingObject* moving_obstacle_osi){
  
  double x_utm, y_utm, x_vel, y_vel, x_accel, y_accel;
  double ego_heading = this->GetEgoVehicleHeading();
  double morai_tm_x_offset = this->GetMoraiTMOffset()[0];
  double morai_tm_y_offset = this->GetMoraiTMOffset()[1];

  moving_obstacle_osi->mutable_id()->set_value(obstacle_ros.unique_id);
  moving_obstacle_osi->set_type(osi3::MovingObject::TYPE_PEDESTRIAN);
  //name
  moving_obstacle_osi->add_source_reference()->add_identifier(obstacle_ros.name);
  //size (m)
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_width(obstacle_ros.size.x);
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_length(obstacle_ros.size.y);
  moving_obstacle_osi->mutable_base()->mutable_dimension()->set_height(obstacle_ros.size.z);

  // position
  this->GetGeoCoordConv().Conv(obstacle_ros.position.x + morai_tm_x_offset,
                obstacle_ros.position.y + morai_tm_y_offset,
                x_utm, y_utm);
  moving_obstacle_osi->mutable_base()->mutable_position()->set_x(x_utm);
  moving_obstacle_osi->mutable_base()->mutable_position()->set_y(y_utm);
  // moving_obstacle_osi->mutable_base()->mutable_position()->set_z(obstacle_ros.position.z);

  //heading (degree to rad)
  double obj_heading = math::deg2rad(obstacle_ros.heading);
  moving_obstacle_osi->mutable_base()->mutable_orientation()->set_yaw(obj_heading);

  // To Ego vehicle coordinate
  // velocity
  double local_heading = obj_heading - ego_heading;

  double velocity_2d[2] = {math::kmh2ms(obstacle_ros.velocity.x), math::kmh2ms(obstacle_ros.velocity.y)};
  x_vel = cos(local_heading)*velocity_2d[0]
        - sin(local_heading)*velocity_2d[1];
  y_vel = sin(local_heading)*velocity_2d[0]
        + cos(local_heading)*velocity_2d[1];

  // acceleration
  double accel_2d[2] = {obstacle_ros.acceleration.x, obstacle_ros.acceleration.y};    
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

  moving_obstacle_osi->mutable_base()->mutable_velocity()->set_x(x_vel);
  moving_obstacle_osi->mutable_base()->mutable_velocity()->set_y(y_vel);
                                  

  double tmp_x_accel = x_accel;
  double tmp_y_accel = y_accel;

  x_accel = cos(ego_heading)*tmp_x_accel
            -sin(ego_heading)*tmp_y_accel;

  y_accel = sin(ego_heading)*tmp_x_accel
            +cos(ego_heading)*tmp_y_accel;

  moving_obstacle_osi->mutable_base()->mutable_acceleration()->set_x(x_accel);
  moving_obstacle_osi->mutable_base()->mutable_acceleration()->set_y(y_accel);
}

void SensorDataOSIConverter::StaticObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, 
                                                osi3::StationaryObject* stationary_obstacle_osi){
  
  double x_utm, y_utm, x_vel, y_vel, x_accel, y_accel;
  double ego_heading = this->GetEgoVehicleHeading();
  double morai_tm_x_offset = this->GetMoraiTMOffset()[0];
  double morai_tm_y_offset = this->GetMoraiTMOffset()[1];

  if(stationary_obstacle_osi){
    stationary_obstacle_osi->mutable_id()->set_value(obstacle_ros.unique_id);
    // name
    stationary_obstacle_osi->add_source_reference()->add_identifier(obstacle_ros.name);
    // size (m)
    stationary_obstacle_osi->mutable_base()->mutable_dimension()->set_length(obstacle_ros.size.x);
    stationary_obstacle_osi->mutable_base()->mutable_dimension()->set_width(obstacle_ros.size.y);
    stationary_obstacle_osi->mutable_base()->mutable_dimension()->set_height(obstacle_ros.size.z);

    // position
    this->GetGeoCoordConv().Conv(obstacle_ros.position.x + morai_tm_x_offset,
                  obstacle_ros.position.y + morai_tm_y_offset,
                  x_utm, y_utm);
    stationary_obstacle_osi->mutable_base()->mutable_position()->set_x(x_utm);
    stationary_obstacle_osi->mutable_base()->mutable_position()->set_y(y_utm);
    // stationary_obstacle_osi->mutable_base()->mutable_position()->set_z(obstacle_ros.position.z);

    //heading (degree to rad)
    stationary_obstacle_osi->mutable_base()->mutable_orientation()->set_yaw(math::deg2rad(obstacle_ros.heading));
  }

}

void SensorDataOSIConverter::SetGeoCoordConv(const std::string* hdmap_path){

  if(hdmap_path->substr(hdmap_path->rfind("/")+1) == "Pangyo HD Map"){
    geo_conv_ = CGeoCoordConv(GeoEllips::kGrs80, GeoSystem::kTmMid,
                            GeoEllips::kWgs84, GeoSystem::kUtm52);
  }

  else if(hdmap_path->substr(hdmap_path->rfind("/")+1) == "KATRI HD Map"){
    geo_conv_ = CGeoCoordConv(GeoEllips::kWgs84, GeoSystem::kUtm52,
                            GeoEllips::kWgs84, GeoSystem::kUtm52);
  }
}