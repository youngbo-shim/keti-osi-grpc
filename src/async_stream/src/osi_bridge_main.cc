#include "keti_ros_bridge.h"
#include "autoware_ros_bridge.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "grpc bridge");
  std::string host_name, cmd_host_name, bridge_name;

  host_name = argv[1];
  cmd_host_name = argv[2];
  bridge_name = argv[3];

  std::cout << "host_name : " << host_name << std::endl;
  std::cout << "cmd_host_name : " << cmd_host_name << std::endl;
  std::cout << "bridge_name : " << bridge_name << std::endl;

  OSIBridge *osi_bridge;

  if(bridge_name == "keti"){
    std::cout << "set keti ros bridge" << std::endl;
    osi_bridge = new KetiROSBridge(host_name, cmd_host_name);
  } 
  
  else if(bridge_name == "autoware"){
    std::cout << "set autoware ros bridge" << std::endl;
    osi_bridge = new AutowareROSBridge(host_name, cmd_host_name);
  }
  
  else{
    std::cout << "there is no bridge named " << bridge_name << std::endl;
    return -1;
  }

  osi_bridge->ClientStartListen();
  osi_bridge->ServerStartStream();
  osi_bridge->StartBridge();

  return 0;
}