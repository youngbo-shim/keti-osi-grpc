#include <pybind11/pybind11.h>

#include <ros/ros.h>

#include "osi_bridge.h"
#include "keti_ros_bridge.h"

namespace py = pybind11;

void ros_init(std::string &node_name){
  int argc = 1;
  char* argv[1] = {"osi_bridge"};

  ros::init(argc, argv, node_name);
}

PYBIND11_MODULE(osi_bridge, m){
  m.def("ros_init", &ros_init);

  py::class_<OSIBridge, std::unique_ptr<OSIBridge>>(m, "OSIBridge");
  
  py::class_<KetiROSBridge, OSIBridge, std::unique_ptr<KetiROSBridge>>(m, "KetiROSBridge")
    .def(py::init<std::string, std::string>())
    .def("StartBridge", &KetiROSBridge::StartBridge)
    .def("ClientStartListen", &KetiROSBridge::ClientStartListen)
    .def("ServerStartStream", &KetiROSBridge::ServerStartStream)
    .def("Stop", &KetiROSBridge::Stop)
    .def("CallbackUpdateOffsetParams", &KetiROSBridge::CallbackUpdateOffsetParams)
    .def("CallbackKetiCmd", &KetiROSBridge::CallbackKetiCmd)
    .def("ConvertThread", &KetiROSBridge::ConvertThread)
    .def("PublishThread", &KetiROSBridge::PublishThread);
}