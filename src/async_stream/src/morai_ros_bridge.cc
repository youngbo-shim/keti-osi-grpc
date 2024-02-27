#include "morai_ros_bridge.h"
#include "morai_msgs/CtrlCmd.h"

MoraiROSBridge::MoraiROSBridge(std::string client_ip_address, std::string server_ip_address) : OSIBridge(client_ip_address, server_ip_address)
{
  pub_morai_cmd_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

  std::cout << "finished" << std::endl;

  is_initialized_ = true;
}

void MoraiROSBridge::StartBridge(){
  if(!is_initialized_){
    std::cout << "KETI ROS Bridge has Not been initiated!" << std::endl;
    return;
  }

  publish_thread_ = std::thread(&MoraiROSBridge::PublishThread, this);
  ros::spin();
}

void MoraiROSBridge::Stop(){
  publish_thread_.join();
}

void MoraiROSBridge::PublishThread(){ 
  while(1){
    SensorView sensor_view;
    if(!sensor_view_buf_.empty()){
        std::lock_guard<std::mutex> lock(sensor_view_mutex_);
        sensor_view.CopyFrom(sensor_view_buf_.front());
        // std::cout << "cmd client sensor_view_size : " << sensor_view_buf_.size() << std::endl;
        sensor_view_buf_.pop();
    } else {
      continue;
    }
    auto host_vehicle_data = sensor_view.host_vehicle_data();
    morai_msgs::CtrlCmd cmd;
    cmd.steering = host_vehicle_data.vehicle_steering().vehicle_steering_wheel().angle();
    if(host_vehicle_data.vehicle_motion().has_acceleration()){
      cmd.longlCmdType = 3;
      cmd.acceleration = host_vehicle_data.vehicle_motion().acceleration().x();
    }
    else if(host_vehicle_data.vehicle_motion().has_velocity()){
      cmd.longlCmdType = 2;
      cmd.velocity = host_vehicle_data.vehicle_motion().velocity().x();
    }
    else if(host_vehicle_data.vehicle_powertrain().has_pedal_position_acceleration() ||
            host_vehicle_data.vehicle_brake_system().has_pedal_position_brake()){
      cmd.longlCmdType = 1;
      cmd.accel = host_vehicle_data.vehicle_powertrain().pedal_position_acceleration();
      cmd.brake = host_vehicle_data.vehicle_brake_system().pedal_position_brake();
    }

    // std::cout << "publish cmd" << std::endl;

    pub_morai_cmd_.publish(cmd);
  }
}
