#include "apollo_ros_converter.h"

ApolloROSConverter::ApolloROSConverter(){}

// std::pair<size_t,VelodyneScan> ApolloROSConverter::ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
//                                                       int lidar_id, size_t seq){
//   VelodyneScan cloud_msg;
//   // VelodynePacket* raw = reinterpret_cast<VelodynePacket*>(&cloud_msg.firing_pkts[0]);
//   // return std::make_pair(seq,cloud_msg);

//   // Set header frame ID and timestamp
//   cloud_msg->mutable_header()->set_frame_id("lidar" + std::to_string(lidar_id));
//   cloud_msg->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

//   // Set model and mode (assuming default values for now)
//   cloud_msg->set_model(apollo::drivers::velodyne::VLS128);
//   cloud_msg->set_mode(apollo::drivers::velodyne::STRONGEST);

//   // Set basetime (assuming 0 for now)
//   cloud_msg->set_basetime(ros::Time::now().toSec());

//   // Iterate over points in the PointCloud
//   for (int i = 0; i < in_msg->point_size(); ++i) {
//     const auto& point = in_msg->point(i);

//     // Create a VelodynePacket for each point
//     auto* packet = cloud_msg->add_firing_pkts();
//     packet->set_stamp(point.timestamp()); // Assuming timestamp exists in PointCloud
//     packet->set_data(/* Encode point data here */); // You need to encode point data properly
//   }
// }

// // velodyne packet 데이터 생성하기
// void Velodyne128Parser::Pack(const std::shared_ptr<PointCloud>& pc,
//                              VelodynePacket* pkt) {
//   RawPacket raw_packet;

//   // Assuming basetime is provided elsewhere
//   double basetime = pc->header().timestamp_sec(); 

//   // Assuming some default values for azimuth and azimuth_next
//   uint16_t azimuth = 0;
//   uint16_t azimuth_next = 0;

//   for (int i = 0; i < pc->point_size(); ++i) {
//     const auto& point = pc->point(i);

//     // Assuming azimuth calculation based on point's coordinates
//     azimuth = CalculateAzimuth(point.x(), point.y());

//     // Add azimuth to raw packet
//     raw_packet.blocks[i / SCANS_PER_BLOCK].rotation = azimuth;

//     // Encode other point attributes into Velodyne packet
//     EncodePoint(point, basetime, &raw_packet.blocks[i / SCANS_PER_BLOCK]);

//     // Assuming azimuth_next calculation based on next point's coordinates
//     azimuth_next = CalculateAzimuth(next_point.x(), next_point.y());
//   }

//   // Convert raw_packet to byte array and set it to VelodynePacket data field
//   std::string raw_data;
//   raw_packet.SerializeToString(&raw_data);
//   pkt->set_data(raw_data);
// }

std::pair<size_t,CompressedImage> ApolloROSConverter::ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                                int camera_id, size_t seq){
                                                                  
  CompressedImage img;
  double current_time = ros::Time::now().toSec();
  img.mutable_header()->set_timestamp_sec(current_time);
  img.set_format("png");
  img.set_data({camera_sensor_view->image_data().begin(),
              camera_sensor_view->image_data().end()});
  return std::make_pair(seq,img);
}

std::pair<size_t, geometry_msgs::TwistStamped> ApolloROSConverter::ProcEgoVehicleSpeed(const HostVehicleData& ego_vehicle_state_view, size_t ego_vehicle_state_seq){

  geometry_msgs::TwistStamped ego_vehicle_speed;
  ego_vehicle_speed.header.stamp = ros::Time::now();
  ego_vehicle_speed.header.frame_id = "base_link";
  ego_vehicle_speed.twist.linear.x = ego_vehicle_state_view.vehicle_motion().velocity().x();
  
  return std::make_pair(ego_vehicle_state_seq, ego_vehicle_speed);
}

std::pair<size_t, morai_msgs::GPSMessage> ApolloROSConverter::ProcGps(const HostVehicleData& gps_sensor_view, size_t gps_seq){

  morai_msgs::GPSMessage gps_msg;
  gps_msg.header.frame_id = "gps";
  gps_msg.header.stamp = ros::Time::now();

  gps_msg.latitude = gps_sensor_view.vehicle_motion().position().x();
  gps_msg.longitude = gps_sensor_view.vehicle_motion().position().y();
  gps_msg.altitude = gps_sensor_view.vehicle_motion().position().z();

  gps_msg.eastOffset = this->GetoffsetX();
  gps_msg.northOffset = this->GetoffsetY();

  return std::make_pair(gps_seq, gps_msg);
    
}

std::pair<size_t, Chassis> ApolloROSConverter::ProcVehicleChassis(const HostVehicleData& chassis_sensor_view, size_t seq){

  Chassis chassis_msg;
  double current_time = ros::Time::now().toSec();
  chassis_msg.mutable_header()->set_timestamp_sec(current_time);
  chassis_msg.mutable_header()->set_sequence_num(seq);
  // gear mode
  chassis_msg.set_gear_location(Chassis::GearPosition(chassis_sensor_view.vehicle_powertrain().gear_transmission()));
  // vehicle driving mode
  if(chassis_sensor_view.vehicle_automated_driving_function_size()){
    if(!chassis_sensor_view.vehicle_automated_driving_function(0).custom_name().compare("MANUAL")){
      chassis_msg.set_driving_mode(Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);
      // SetEgoDrivingMode(DRIVING_MANUAL_MODE);
    }
    else if(!chassis_sensor_view.vehicle_automated_driving_function(0).custom_name().compare("AUTO")){
      chassis_msg.set_driving_mode(Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
      // SetEgoDrivingMode(DRIVING_AUTO_MODE);
    }
    else
      std::cout << "Vehicle_automated_driving_function custom name is not set" << std::endl;
  }

  chassis_msg.set_engine_rpm(750.0);
  chassis_msg.set_speed_mps(chassis_sensor_view.vehicle_motion().velocity().x());
  chassis_msg.set_throttle_percentage(chassis_sensor_view.vehicle_powertrain().pedal_position_acceleration());
  chassis_msg.set_brake_percentage(chassis_sensor_view.vehicle_brake_system().pedal_position_brake());
  chassis_msg.set_steering_percentage(chassis_sensor_view.vehicle_steering().vehicle_steering_wheel().angle() / MORAI_VEHICLE_MAX_STEER_ANGLE_IN_DEG * -100.0);
  chassis_msg.mutable_chassis_gps()->set_latitude(chassis_sensor_view.vehicle_localization().geodetic_position().latitude());
  chassis_msg.mutable_chassis_gps()->set_longitude(chassis_sensor_view.vehicle_localization().geodetic_position().longitude());
  chassis_msg.mutable_chassis_gps()->set_altitude(chassis_sensor_view.vehicle_localization().geodetic_position().altitude());
  chassis_msg.mutable_chassis_gps()->set_hdop(0.1);
  chassis_msg.mutable_chassis_gps()->set_vdop(0.1);
  chassis_msg.mutable_chassis_gps()->set_quality(apollo::canbus::GpsQuality::FIX_3D);
  chassis_msg.mutable_chassis_gps()->set_num_satellites(15);
  
  return std::make_pair(seq, chassis_msg);
}

std::pair<size_t, GnssBestPose> ApolloROSConverter::ProcBestPose(const HostVehicleData& gps_sensor_view, size_t seq){

  GnssBestPose gnss_best_pose_msg;
  double current_time = ros::Time::now().toSec();
  gnss_best_pose_msg.mutable_header()->set_timestamp_sec(current_time);
  gnss_best_pose_msg.mutable_header()->set_sequence_num(seq);
  gnss_best_pose_msg.set_measurement_time(current_time);
  gnss_best_pose_msg.set_sol_status(apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
  gnss_best_pose_msg.set_latitude(gps_sensor_view.vehicle_localization().geodetic_position().latitude());
  gnss_best_pose_msg.set_longitude(gps_sensor_view.vehicle_localization().geodetic_position().longitude());
  gnss_best_pose_msg.set_height_msl(gps_sensor_view.vehicle_localization().geodetic_position().altitude());
  gnss_best_pose_msg.set_undulation(0.0);
  gnss_best_pose_msg.set_datum_id(apollo::drivers::gnss::DatumId::WGS84);
  gnss_best_pose_msg.set_latitude_std_dev(0.01);
  gnss_best_pose_msg.set_longitude_std_dev(0.01);
  gnss_best_pose_msg.set_height_std_dev(0.01);
  gnss_best_pose_msg.set_differential_age(2.0);
  gnss_best_pose_msg.set_solution_age(0.0);
  gnss_best_pose_msg.set_num_sats_tracked(15);
  gnss_best_pose_msg.set_num_sats_in_solution(15);
  gnss_best_pose_msg.set_num_sats_multi(12);
  gnss_best_pose_msg.set_extended_solution_status(33);
  gnss_best_pose_msg.set_galileo_beidou_used_mask(0);
  gnss_best_pose_msg.set_gps_glonass_used_mask(51);

  return std::make_pair(seq, gnss_best_pose_msg);
}
std::pair<size_t, Imu> ApolloROSConverter::ProcImu(const HostVehicleData& imu_sensor_view, size_t seq){

  Imu imu_msg;
  double current_time = ros::Time::now().toSec();
  imu_msg.mutable_header()->set_timestamp_sec(current_time);
  imu_msg.mutable_header()->set_sequence_num(seq);
  imu_msg.set_measurement_time(current_time);
  imu_msg.set_measurement_span(1/50.0);
  imu_msg.mutable_linear_acceleration()->set_x(imu_sensor_view.vehicle_motion().acceleration().x());
  imu_msg.mutable_linear_acceleration()->set_y(imu_sensor_view.vehicle_motion().acceleration().y());
  imu_msg.mutable_linear_acceleration()->set_z(imu_sensor_view.vehicle_motion().acceleration().z());
  imu_msg.mutable_angular_velocity()->set_x(imu_sensor_view.vehicle_motion().orientation_rate().roll());
  imu_msg.mutable_angular_velocity()->set_y(imu_sensor_view.vehicle_motion().orientation_rate().pitch());
  imu_msg.mutable_angular_velocity()->set_z(imu_sensor_view.vehicle_motion().orientation_rate().yaw());
  
  return std::make_pair(seq, imu_msg);
  
}

std::pair<size_t, CorrectedImu> ApolloROSConverter::ProcCorImu(const HostVehicleData& corimu_sensor_view, size_t seq){

  CorrectedImu corimu_msg;
  double current_time = ros::Time::now().toSec();
  corimu_msg.mutable_header()->set_timestamp_sec(current_time);
  corimu_msg.mutable_header()->set_sequence_num(seq);
  corimu_msg.mutable_imu()->mutable_euler_angles()->set_x(corimu_sensor_view.vehicle_motion().orientation_rate().roll());
  corimu_msg.mutable_imu()->mutable_euler_angles()->set_y(corimu_sensor_view.vehicle_motion().orientation_rate().pitch());
  corimu_msg.mutable_imu()->mutable_euler_angles()->set_z(corimu_sensor_view.vehicle_motion().orientation_rate().yaw());
  corimu_msg.mutable_imu()->set_heading(corimu_sensor_view.vehicle_motion().orientation().yaw());

  return std::make_pair(seq, corimu_msg);

}

std::pair<size_t, Gps> ApolloROSConverter::ProcOdometry(const HostVehicleData& odometry_sensor_view, size_t seq){
  
  Gps odom_msg;
  double current_time = ros::Time::now().toSec();
  double heading = odometry_sensor_view.vehicle_motion().orientation().yaw() * 180 / M_PI;
  double tm_heading = heading - 90.0 <= -180.0 ? 
                      heading + 360.0 :
                      heading - 90.0;
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, tm_heading / 180 * M_PI );
  odom_msg.mutable_header()->set_timestamp_sec(current_time);
  odom_msg.mutable_header()->set_sequence_num(seq);
  odom_msg.mutable_localization()->mutable_position()->set_x(odometry_sensor_view.vehicle_motion().position().x());
  odom_msg.mutable_localization()->mutable_position()->set_y(odometry_sensor_view.vehicle_motion().position().y());
  odom_msg.mutable_localization()->mutable_linear_velocity()->set_x(odometry_sensor_view.vehicle_motion().velocity().x());
  odom_msg.mutable_localization()->mutable_linear_velocity()->set_y(odometry_sensor_view.vehicle_motion().velocity().y());
  odom_msg.mutable_localization()->mutable_linear_velocity()->set_z(odometry_sensor_view.vehicle_motion().velocity().z());
  odom_msg.mutable_localization()->set_heading(tm_heading);
  odom_msg.mutable_localization()->mutable_orientation()->set_qx(q.getX());
  odom_msg.mutable_localization()->mutable_orientation()->set_qy(q.getY());
  odom_msg.mutable_localization()->mutable_orientation()->set_qz(q.getZ());
  odom_msg.mutable_localization()->mutable_orientation()->set_qw(q.getW());
  return std::make_pair(seq, odom_msg);
}

std::pair<size_t, InsStat> ApolloROSConverter::ProcIns(size_t seq){

  InsStat ins_msg;
  double current_time = ros::Time::now().toSec();
  ins_msg.mutable_header()->set_timestamp_sec(current_time);
  ins_msg.mutable_header()->set_sequence_num(seq);
  ins_msg.mutable_header()->set_frame_id("gps");
  ins_msg.set_ins_status(3);
  ins_msg.set_pos_type(56);

  return std::make_pair(seq, ins_msg);
}

std::pair<size_t, TrafficLightDetection> ApolloROSConverter::ProcTrafficLightDetection(const HostVehicleData& traffic_light_view, size_t seq){

  TrafficLightDetection traffic_light_msg;
  double current_time = ros::Time::now().toSec();
  traffic_light_msg.mutable_header()->set_timestamp_sec(current_time);
  traffic_light_msg.mutable_header()->set_sequence_num(seq);
  traffic_light_msg.mutable_header()->set_camera_timestamp(current_time * std::pow(10,6));
  traffic_light_msg.set_contain_lights(1);
  return std::make_pair(seq, traffic_light_msg);

}

// std::pair<size_t, PerceptionObstacles> ApolloROSConverter::ProcPerceptionObstacles(const HostVehicleData& perception_obs_view, size_t seq){

//   PerceptionObstacles perception_obs_msg;
//   double current_time = ros::Time::now().toSec();
//   perception_obs_msg.mutable_header()->set_timestamp_sec(current_time);
//   perception_obs_msg.mutable_header()->set_sequence_num(seq);

//   return std::make_pair(seq, perception_obs_msg);

// }

std::pair<size_t, PerceptionObstacles> ApolloROSConverter::ProcPerceptionObstacles(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                    std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                      size_t seq){
  PerceptionObstacles obstacles;
  double current_time = ros::Time::now().toSec();
  obstacles.mutable_header()->set_timestamp_sec(current_time);
  obstacles.mutable_header()->set_sequence_num(seq);

  // obstacles.header.frame_id = "base_link";
  // obstacles.header.timestamp_sec = ros::Time::now().toSec();

  if(osi_moving_objs.get()->size() == 1 && osi_moving_objs.get()->Get(0).id().value() == 0 &&
      osi_stationary_objs.get()->size() == 1 && osi_stationary_objs.get()->Get(0).id().value() == 0 ){

    return std::make_pair(seq, obstacles);
  }

  for(int i = 0 ; i < osi_moving_objs.get()->size() ; i++){
    auto osi_moving_obj = osi_moving_objs.get()->Get(i);
    auto obs = obstacles.add_perception_obstacle();
    // id
    obs->set_id(osi_moving_obj.id().value());
    // type
    if(osi_moving_obj.type() == osi3::MovingObject::TYPE_PEDESTRIAN){
      obs->set_type(apollo::perception::PerceptionObstacle::Type::PerceptionObstacle_Type_PEDESTRIAN);
    }
    else if(osi_moving_obj.type() == osi3::MovingObject::TYPE_VEHICLE){
      switch(osi_moving_obj.vehicle_classification().type()){
        case osi3::MovingObject::VehicleClassification::TYPE_BICYCLE:
        case osi3::MovingObject::VehicleClassification::TYPE_MOTORBIKE:
          obs->set_type(apollo::perception::PerceptionObstacle::Type::PerceptionObstacle_Type_BICYCLE);
          break;
        default:
          obs->set_type(apollo::perception::PerceptionObstacle::Type::PerceptionObstacle_Type_VEHICLE);
          break;
      }
    }
    // size (m)
    obs->set_length(osi_moving_obj.base().dimension().length());
    obs->set_width(osi_moving_obj.base().dimension().width());
    obs->set_height(osi_moving_obj.base().dimension().height());
    // heading
    obs->set_theta(osi_moving_obj.base().orientation().yaw());
    // position
    obs->mutable_position()->set_x(osi_moving_obj.base().position().x());
    obs->mutable_position()->set_y(osi_moving_obj.base().position().y());
    obs->mutable_position()->set_z(osi_moving_obj.base().position().z());
    // velocity
    obs->mutable_velocity()->set_x(osi_moving_obj.base().velocity().x());
    obs->mutable_velocity()->set_y(osi_moving_obj.base().velocity().y());
    // acceleration
    obs->mutable_acceleration()->set_x(osi_moving_obj.base().acceleration().x());
    obs->mutable_acceleration()->set_y(osi_moving_obj.base().acceleration().y());
  }

  for(int i = 0 ; i < osi_stationary_objs.get()->size() ; i++){
    auto osi_stationary_obj = osi_stationary_objs.get()->Get(i);
    auto obs = obstacles.add_perception_obstacle();
    // id
    obs->set_id(osi_stationary_obj.id().value());
    // type
    obs->set_type(apollo::perception::PerceptionObstacle::Type::PerceptionObstacle_Type_UNKNOWN_UNMOVABLE);
    // size (m)
    obs->set_length(osi_stationary_obj.base().dimension().length());
    obs->set_width(osi_stationary_obj.base().dimension().width());
    obs->set_height(osi_stationary_obj.base().dimension().height());
    // heading
    obs->set_theta(osi_stationary_obj.base().orientation().yaw());
 
    // position
    obs->mutable_position()->set_x(osi_stationary_obj.base().position().x());
    obs->mutable_position()->set_y(osi_stationary_obj.base().position().y());
    obs->mutable_position()->set_z(osi_stationary_obj.base().position().z());
    // velocity
    obs->mutable_velocity()->set_x(0.0);
    obs->mutable_velocity()->set_y(0.0);
    // acceleration
    obs->mutable_acceleration()->set_x(0.0);
    obs->mutable_acceleration()->set_y(0.0);
  }

  return std::make_pair(seq,obstacles);
}


std_msgs::ColorRGBA ApolloROSConverter::colorCategory10(int i) {
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  switch (i % 20) {
    case 0: {
      c.r = 0.121569;
      c.g = 0.466667;
      c.b = 0.705882;
    } break;
    case 1: {
      c.r = 0.682353;
      c.g = 0.780392;
      c.b = 0.909804;
    } break;
    case 2: {
      c.r = 1.000000;
      c.g = 0.498039;
      c.b = 0.054902;
    } break;
    case 3: {
      c.r = 1.000000;
      c.g = 0.733333;
      c.b = 0.470588;
    } break;
    case 4: {
      c.r = 0.172549;
      c.g = 0.627451;
      c.b = 0.172549;
    } break;
    case 5: {
      c.r = 0.596078;
      c.g = 0.874510;
      c.b = 0.541176;
    } break;
    case 6: {
      c.r = 0.839216;
      c.g = 0.152941;
      c.b = 0.156863;
    } break;
    case 7: {
      c.r = 1.000000;
      c.g = 0.596078;
      c.b = 0.588235;
    } break;
    case 8: {
      c.r = 0.580392;
      c.g = 0.403922;
      c.b = 0.741176;
    } break;
    case 9: {
      c.r = 0.772549;
      c.g = 0.690196;
      c.b = 0.835294;
    } break;
    case 10: {
      c.r = 0.549020;
      c.g = 0.337255;
      c.b = 0.294118;
    } break;
  }
  return c;
}
