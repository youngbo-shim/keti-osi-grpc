#include "autoware_ros_converter.h"

AutowareROSConverter::AutowareROSConverter(){}

std::pair<size_t,sensor_msgs::PointCloud2> AutowareROSConverter::ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
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

  return std::make_pair(seq,cloud_msg);
}

std::pair<size_t,sensor_msgs::CompressedImage> AutowareROSConverter::ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                          int camera_id, size_t seq){
  sensor_msgs::CompressedImage img;
  img.header.stamp = ros::Time::now();
  img.format = "png";
  img.data = {camera_sensor_view->image_data().begin(),
              camera_sensor_view->image_data().end()};
  return std::make_pair(seq,img);
}

std::pair<size_t,autoware_msgs::DetectedObjectArray> AutowareROSConverter::ProcObj(std::shared_ptr<RepeatedPtrField<MovingObject>> osi_moving_objs,
                                                                      std::shared_ptr<RepeatedPtrField<StationaryObject>> osi_stationary_objs,
                                                                      size_t seq){
  autoware_msgs::DetectedObjectArray obstacles;

  obstacles.header.frame_id = "world";
  obstacles.header.stamp = ros::Time::now();

  if(osi_moving_objs.get()->size() == 1 && osi_moving_objs.get()->Get(0).id().value() == 0 &&
      osi_stationary_objs.get()->size() == 1 && osi_stationary_objs.get()->Get(0).id().value() == 0 ){

    return std::make_pair(seq,obstacles);
  }

  for(int i = 0 ; i < osi_moving_objs.get()->size() ; i++){
    auto osi_moving_obj = osi_moving_objs.get()->Get(i);
    autoware_msgs::DetectedObject obj;
    // id
    obj.id = osi_moving_obj.id().value();
    // type
    if(osi_moving_obj.type() == osi3::MovingObject::TYPE_PEDESTRIAN){
      obj.label = "PEDESTRIAN";
      obj.color = colorCategory10(static_cast<int>(osi3::MovingObject::TYPE_PEDESTRIAN));
    }
    else if(osi_moving_obj.type() == osi3::MovingObject::TYPE_VEHICLE){
      switch(osi_moving_obj.vehicle_classification().type()){
        case osi3::MovingObject::VehicleClassification::TYPE_BICYCLE:
        case osi3::MovingObject::VehicleClassification::TYPE_MOTORBIKE:
          obj.label = "BICYCLE";
          obj.color = colorCategory10(static_cast<int>(osi3::MovingObject::TYPE_OTHER));
          break;
        default:
          obj.label = "VEHICLE";
          obj.color = colorCategory10(static_cast<int>(osi3::MovingObject::TYPE_VEHICLE));
          break;
      }
    }

    // position
    obj.pose.position.x = osi_moving_obj.base().position().x() - this->GetoffsetX();
    obj.pose.position.y = osi_moving_obj.base().position().y() - this->GetoffsetY();
    obj.pose.position.z = osi_moving_obj.base().position().z() / 2.0;

    // heading
    tf2::Quaternion quat;
    quat.setRPY( 0.0, 0.0, osi_moving_obj.base().orientation().yaw());
    obj.pose.orientation.x = quat[0];
    obj.pose.orientation.y = quat[1];
    obj.pose.orientation.z = quat[2];
    obj.pose.orientation.w = quat[3];

    // size (m)
    obj.dimensions.x = osi_moving_obj.base().dimension().length();
    obj.dimensions.y = osi_moving_obj.base().dimension().width();
    obj.dimensions.z = osi_moving_obj.base().dimension().height();
    
    // velocity
    obj.velocity.linear.x = osi_moving_obj.base().velocity().x();
    obj.velocity.linear.y = osi_moving_obj.base().velocity().y();
    
    // acceleration
    obj.acceleration.linear.x = osi_moving_obj.base().acceleration().x();
    obj.acceleration.linear.y = osi_moving_obj.base().acceleration().y();

    obj.header.frame_id = "world";
    obj.header.stamp = ros::Time::now();
    obj.convex_hull.header.frame_id = obj.header.frame_id;
    obj.convex_hull.header.stamp = obj.header.stamp;
    geometry_msgs::Point32 obj_point;
    obj_point.x = obj.pose.position.x;
    obj_point.y = obj.pose.position.y;
    obj_point.z = obj.pose.position.z;
    obj.convex_hull.polygon.points.push_back(obj_point);
    obj.score = 1;
    obj.valid = true;
    obj.space_frame = "world";
    obj.pose_reliable = true;

    obstacles.objects.push_back(obj);
  }

  for(int i = 0 ; i < osi_stationary_objs.get()->size() ; i++){
    auto osi_stationary_obj = osi_stationary_objs.get()->Get(i);
    autoware_msgs::DetectedObject obj;
    // id
    obj.id = osi_stationary_obj.id().value();
    
    // type
    obj.label = "UNKNOWN_UNMOVABLE";
    obj.color = colorCategory10(static_cast<int>(osi3::MovingObject::TYPE_UNKNOWN));

    // position
    obj.pose.position.x = osi_stationary_obj.base().position().x() - this->GetoffsetX();
    obj.pose.position.y = osi_stationary_obj.base().position().y() - this->GetoffsetY();
    obj.pose.position.z = osi_stationary_obj.base().position().z() / 2.0;

    // heading
    tf2::Quaternion quat;
    quat.setRPY( 0.0, 0.0, osi_stationary_obj.base().orientation().yaw());
    obj.pose.orientation.x = quat[0];
    obj.pose.orientation.y = quat[1];
    obj.pose.orientation.z = quat[2];
    obj.pose.orientation.w = quat[3];    

    // size (m)
    obj.dimensions.x = osi_stationary_obj.base().dimension().length();
    obj.dimensions.y = osi_stationary_obj.base().dimension().width();
    obj.dimensions.z = osi_stationary_obj.base().dimension().height();
    
    // velocity
    obj.velocity.linear.x = 0.0;
    obj.velocity.linear.y = 0.0;
    
    // acceleration
    obj.acceleration.linear.x = 0.0;
    obj.acceleration.linear.y = 0.0;

    obj.header.frame_id = "world";
    obj.header.stamp = ros::Time::now();
    obj.convex_hull.header.frame_id = obj.header.frame_id;
    obj.convex_hull.header.stamp = obj.header.stamp;
    geometry_msgs::Point32 obj_point;
    obj_point.x = obj.pose.position.x;
    obj_point.y = obj.pose.position.y;
    obj_point.z = obj.pose.position.z;
    obj.convex_hull.polygon.points.push_back(obj_point);
    obj.score = 1;
    obj.valid = true;
    obj.space_frame = "world";
    obj.pose_reliable = true;

    obstacles.objects.push_back(obj);
  }

  return std::make_pair(seq,obstacles);
}

std::pair<size_t,sensor_msgs::Imu> AutowareROSConverter::ProcImu(const HostVehicleData& imu_sensor_view, size_t imu_seq){

  tf2::Quaternion quat;
  quat.setRPY( imu_sensor_view.vehicle_motion().orientation().roll(),
                      imu_sensor_view.vehicle_motion().orientation().pitch(),
                      imu_sensor_view.vehicle_motion().orientation().yaw());

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = ros::Time::now();

  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  imu_msg.angular_velocity.x = imu_sensor_view.vehicle_motion().orientation_rate().roll();
  imu_msg.angular_velocity.y = imu_sensor_view.vehicle_motion().orientation_rate().pitch();
  imu_msg.angular_velocity.z = imu_sensor_view.vehicle_motion().orientation_rate().yaw();

  imu_msg.linear_acceleration.x = imu_sensor_view.vehicle_motion().acceleration().x();
  imu_msg.linear_acceleration.y = imu_sensor_view.vehicle_motion().acceleration().y();
  imu_msg.linear_acceleration.z = imu_sensor_view.vehicle_motion().acceleration().z();

  return std::make_pair(imu_seq, imu_msg);
}

std::pair<size_t, geometry_msgs::PoseStamped> AutowareROSConverter::ProcEgoVehicleState(const HostVehicleData& ego_vehicle_state_view, size_t ego_vehicle_state_seq){

  tf2::Quaternion quat;
  quat.setRPY( ego_vehicle_state_view.vehicle_motion().orientation().roll(),
              ego_vehicle_state_view.vehicle_motion().orientation().pitch(),
              ego_vehicle_state_view.vehicle_motion().orientation().yaw());

  geometry_msgs::PoseStamped ego_vehicle_state_msg;
  ego_vehicle_state_msg.header.frame_id = "map";
  ego_vehicle_state_msg.header.stamp = ros::Time::now();

  ego_vehicle_state_msg.pose.position.x = ego_vehicle_state_view.vehicle_motion().position().x() - this->GetoffsetX();
  ego_vehicle_state_msg.pose.position.y = ego_vehicle_state_view.vehicle_motion().position().y() - this->GetoffsetY();
  ego_vehicle_state_msg.pose.position.z = ego_vehicle_state_view.vehicle_motion().position().z();
  ego_vehicle_state_msg.pose.orientation.x = quat.x();
  ego_vehicle_state_msg.pose.orientation.y = quat.y();
  ego_vehicle_state_msg.pose.orientation.z = quat.z();
  ego_vehicle_state_msg.pose.orientation.w = quat.w();

  return std::make_pair(ego_vehicle_state_seq, ego_vehicle_state_msg);
  
}

std::pair<size_t, geometry_msgs::TwistStamped> AutowareROSConverter::ProcEgoVehicleSpeed(const HostVehicleData& ego_vehicle_state_view, size_t ego_vehicle_state_seq){

  geometry_msgs::TwistStamped ego_vehicle_speed;
  ego_vehicle_speed.header.stamp = ros::Time::now();
  ego_vehicle_speed.header.frame_id = "base_link";
  ego_vehicle_speed.twist.linear.x = ego_vehicle_state_view.vehicle_motion().velocity().x();
  
  return std::make_pair(ego_vehicle_state_seq, ego_vehicle_speed);
}

std::pair<size_t, morai_msgs::GPSMessage> AutowareROSConverter::ProcGps(const HostVehicleData& gps_sensor_view, size_t gps_seq){

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

std_msgs::ColorRGBA AutowareROSConverter::colorCategory10(int i) {
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
