#include "OSMPDummySensor.h"

int main(int argc, char** argv){

  osi3::SensorView sensor_view;
  auto lidar_osi = sensor_view.add_lidar_sensor_view();

  size_t num_of_rays = 1;    // ray 1개
  size_t width = 1, height = 10;
  auto lidar_view_configuration = lidar_osi->mutable_view_configuration();
  lidar_view_configuration->set_number_of_rays_vertical(num_of_rays);
  lidar_view_configuration->set_number_of_rays_horizontal(width/num_of_rays);
  lidar_view_configuration->set_num_of_pixels(width*height);
  lidar_view_configuration->mutable_sensor_id()->set_value(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_x(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_y(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_position()->set_z(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_roll(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_pitch(0);
  lidar_view_configuration->mutable_mounting_position()->mutable_orientation()->set_yaw(0);

  for ( size_t i = 0; i < width * height; ) {
    double x_value = 10, y_value = 10, z_value = 0., intensity_value = 200.;
    double magnitude_direction = std::sqrt(x_value*x_value + y_value*y_value + z_value*z_value);              
    
    // Add direction
    auto direction = lidar_view_configuration->add_directions();
    direction->set_x(x_value/magnitude_direction);
    direction->set_y(y_value/magnitude_direction);
    direction->set_z(z_value/magnitude_direction);

    int speed_of_light = 299792458; // m/s
    auto reflection = lidar_osi->add_reflection();
    reflection->set_time_of_flight(magnitude_direction/speed_of_light);
    reflection->set_signal_strength(intensity_value);

    i++;
  }

  fmi2CallbackFunctions callbacks = {
    .logger = NULL,
    .allocateMemory = NULL,
    .freeMemory = NULL,
    .stepFinished = NULL,
    .componentEnvironment = NULL
  };

  COSMPDummySensor osmp_lidar("LiDAR", fmi2CoSimulation, "1", "", &callbacks, false, false);    
  
  int size = sensor_view.ByteSizeLong();
  void* buffer = malloc(size);
  sensor_view.SerializeToArray(buffer, size);  

  std::cout << "bytes : " << sensor_view.ByteSizeLong() << std::endl;

  size_t num_var = 3;
  fmi2ValueReference vr_in[num_var] = { FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX,
                                        FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX,
                                        FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX };

  int address_hi = 0, address_lo = 0;
  osmp_lidar.encode_pointer_to_integer(buffer, address_hi, address_lo);
  fmi2Integer value[num_var] = { address_lo, address_hi, size };

  std::string* tmp = (std::string*)osmp_lidar.decode_integer_to_pointer(address_hi, address_lo);

  std::cout << "address_hi, address_lo : " << buffer << ", " << tmp << ", " << address_hi << ", " << address_lo << std::endl;

  osmp_lidar.SetInteger(vr_in, num_var, value);
  
  // 현재 시간 가져오기
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  double seconds = std::chrono::duration<double>(duration).count();

  fmi2Real currentCommunicationPoint = seconds;
  fmi2Real communicationStepSize = 0.1;
  fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component = false;

  osmp_lidar.DoStep( currentCommunicationPoint,
                     communicationStepSize,
                     noSetFMUStatePriorToCurrentPointfmi2Component);

  fmi2ValueReference vr_out[num_var] = { FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX,
                                         FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX,
                                         FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX };

  fmi2Integer out_value[num_var];
  osmp_lidar.GetInteger(vr_out, num_var, out_value);

  auto out_buffer = osmp_lidar.decode_integer_to_pointer(out_value[1], out_value[0]);

  osi3::SensorData sensor_data;
  sensor_data.ParseFromArray(out_buffer, out_value[2]);

  auto lidar_sensor_view = sensor_data.sensor_view(0).lidar_sensor_view(0);
  
  std::cout << "out_value 0, 1, 2 : " << out_value[0] << ", " << out_value[1] << ", " << out_value[2] << ", " << sensor_data.sensor_view().size() << std::endl;

  int speed_of_light = 299792458; // m/s
  for ( size_t i = 0; i < lidar_sensor_view.view_configuration().directions_size(); i++ ){
    auto direction = lidar_sensor_view.view_configuration().directions()[i];
    auto reflection = lidar_sensor_view.reflection()[i];

    double magnitude = reflection.time_of_flight()*speed_of_light;

    unsigned char bytes[4];
    float x = magnitude*direction.x();    
    float y = magnitude*direction.y();    
    float z = magnitude*direction.z();    
    float intensity = reflection.signal_strength();

    std::cout.precision(2);
    std::cout << std::fixed << "x, y, z, i : " << x << ", " << y << ", " << z << ", " << intensity << std::endl;
  }

  return 0;
}