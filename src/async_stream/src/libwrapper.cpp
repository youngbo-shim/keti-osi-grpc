#include <pybind11/pybind11.h>
#include "keti_ros_bridge.cc"

namespace py = pybind11;

PYBIND11_MODULE(client_lib, m){
  // py::class_<OSIBridge, std::unique_ptr<OSIBridge>>(m, "OSIBridge")
  //   .def(py::init<std::string>())
  //   .def("ClientStartListen", &OSIBridge::ClientStartListen)
  //   .def("StartBridge", &OSIBridge::StartBridge)
  //   .def("Stop", &OSIBridge::Stop);
  
  py::class_<KetiROSBridge, OSIBridge, std::unique_ptr<KetiROSBridge>>(m, "KetiROSBridge")
    .def(py::init<std::string>())
    .def("StartBridge", &KetiROSBridge::StartBridge)
    .def("Stop", &KetiROSBridge::Stop)
    .def("CallbackUpdateOffsetParams", &KetiROSBridge::CallbackUpdateOffsetParams)
    .def("ConvertThread", &KetiROSBridge::ConvertThread)
    .def("PublishThread", &KetiROSBridge::PublishThread);
}