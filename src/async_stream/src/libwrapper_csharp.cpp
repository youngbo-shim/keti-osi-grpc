#include "keti_ros_bridge.h"
#include "libwrapper_csharp.h"

extern "C"{

void ros_init(const char* node_name){
  int argc = 1;
  char* argv[1] = {"KetiROSBridgeWrapper"};

  ros::init(argc, argv, node_name);
}

KetiROSBridgeWrapper* KetiROSBridge_Create(const char* host_name, const char* cmd_host_name){
  return reinterpret_cast<KetiROSBridgeWrapper*>(new KetiROSBridge(host_name, cmd_host_name));
}

void KetiROSBridge_Destroy(KetiROSBridgeWrapper* obj){
  delete reinterpret_cast<KetiROSBridge*>(obj);
}

void KetiROSBridge_StartBridge(KetiROSBridgeWrapper* obj){
  reinterpret_cast<KetiROSBridge*>(obj)->StartBridge();
}

void KetiROSBridge_ClientStartListen(KetiROSBridgeWrapper* obj){
  reinterpret_cast<KetiROSBridge*>(obj)->ClientStartListen();
}

void KetiROSBridge_ServerStartStream(KetiROSBridgeWrapper* obj){
  reinterpret_cast<KetiROSBridge*>(obj)->ServerStartStream();
}

}
