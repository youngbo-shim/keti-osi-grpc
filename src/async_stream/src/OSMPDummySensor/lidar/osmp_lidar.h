#include <cmath>
#include <random>
#include <vector>

#include "fmi2Functions.h"
#include "osi_sensorview.pb.h"

class COSMPLiDARSensor {
public:
  COSMPLiDARSensor();
  
  fmi2Status setInputDeserializedOSI(const char* data, size_t size);
  fmi2Status getOutputSerializedOSI(char** data, size_t* size);
  fmi2Status doStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize);

  // TODO(All) :
  // void SensorErrorModeling();
  void addNoise();

private:
  osi3::SensorView sensor_view_;
};