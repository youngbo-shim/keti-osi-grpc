#include "osmp/lidar/osmp_lidar.h"

using osi3::LidarSensorView;

COSMPLiDARSensor::COSMPLiDARSensor() {

}

fmi2Status COSMPLiDARSensor::setInputDeserializedOSI(const char* data, size_t size) {
  if (!sensor_view_.ParseFromArray(data, size)) {
    return fmi2Error;
  }
  return fmi2OK;
}

fmi2Status COSMPLiDARSensor::getOutputSerializedOSI(char** data, size_t* size) {
  std::string serialized = sensor_view_.SerializeAsString();
  *size = serialized.size();
  *data = new char[*size];
  memcpy(*data, serialized.c_str(), *size);
  return fmi2OK;
}

fmi2Status COSMPLiDARSensor::doStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize) {
  addNoise();

  return fmi2OK;
}

void COSMPLiDARSensor::addNoise() {
  if ( sensor_view_.lidar_sensor_view().size() == 0 ) {
    // 에러 메시지
    return;
  }

  auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view_.lidar_sensor_view(0));

  int speed_of_light = 299792458; // m/s
  for ( size_t i = 0; i < lidar_sensor_view->view_configuration().directions_size(); i++ ){
    auto direction = lidar_sensor_view->view_configuration().directions()[i];
    auto reflection = lidar_sensor_view->reflection()[i];

    float magnitude = reflection.time_of_flight()*speed_of_light;
    std::normal_distribution<> distr(0.0, 0.1);
    std::random_device rd;
    std::mt19937 gen(rd());
    float error = distr(gen); 
    magnitude += error;

    reflection.set_time_of_flight(magnitude/speed_of_light);
  }
}

// FMI 함수 구현
extern "C" {
  COSMPLiDARSensor* instance = nullptr;

  FMI2_Export const char* fmi2GetTypesPlatform() { return fmi2TypesPlatform; }
  FMI2_Export const char* fmi2GetVersion() { return fmi2Version; }

  FMI2_Export fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType,
    fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions,
    fmi2Boolean visible, fmi2Boolean loggingOn) {
    instance = new COSMPLiDARSensor();
    return instance;
  }

  FMI2_Export void fmi2FreeInstance(fmi2Component c) {
    delete instance;
    instance = nullptr;
  }

  FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined,
    fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    return fmi2OK;
  }

  FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c) { return fmi2OK; }
  FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c) { return fmi2OK; }

  FMI2_Export fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    return instance->doStep(currentCommunicationPoint, communicationStepSize);
  }

  FMI2_Export fmi2Status fmi2SetInputDeserializedOSI(fmi2Component c, const char* data, size_t size) {
    return instance->setInputDeserializedOSI(data, size);
  }

  FMI2_Export fmi2Status fmi2GetOutputSerializedOSI(fmi2Component c, char** data, size_t* size) {
    return instance->getOutputSerializedOSI(data, size);
  }

  // 기타 필요한 FMI 함수들 구현...
}