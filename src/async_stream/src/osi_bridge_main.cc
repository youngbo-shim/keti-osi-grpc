#include <ros/ros.h>

#include "keti_ros_bridge.cc"
#include "autoware_ros_bridge.cc"


int main(int argc, char** argv){
  ros::init(argc, argv, "grpc_client");
  ros::NodeHandle nh;
  std::string host_name, bridge_name;

  nh.getParam("host_name", host_name);
  nh.getParam("bridge_name", bridge_name);

  OSIBridge *osi_bridge;

  if(bridge_name == "keti"){
    std::cout << "set keti ros bridge" << std::endl;
    osi_bridge = new KetiROSBridge(host_name);
  } 
  
  else if(bridge_name == "autoware"){
    std::cout << "set autoware ros bridge" << std::endl;
    osi_bridge = new AutowareROSBridge(host_name);
  }
  
  else{
    std::cout << "there is no bridge named " << bridge_name << std::endl;
    return -1;
  }

  osi_bridge->ClientStartListen();
  osi_bridge->StartBridge();

  return 0;
}