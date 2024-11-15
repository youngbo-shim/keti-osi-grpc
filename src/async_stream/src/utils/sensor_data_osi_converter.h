#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/alarm.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <radar_msgs/RadarScan.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include "sensorview_rpc.grpc.pb.h"

#include "morai_msgs/RadarDetections.h"
#include "morai_msgs/EgoVehicleStatus.h"
#include "morai_msgs/GPSMessage.h"
#include "morai_msgs/CtrlCmd.h"
#include "morai_msgs/MoraiEventCmdSrv.h"
#include "morai_msgs/ObjectStatusList.h"
#include "morai_msgs/GetTrafficLightStatus.h"
#include "sensor_msgs/Imu.h"
#include "math/math_utils.h"
#include "conversion/coordinate/utm_conversion.h"
#include "conversion/coordinate/tm_conversion.h"
#include "autoware_msgs/VehicleStatus.h"
#include "utils/string_utils.h"

#define SPEED_OF_LIGHT 299792458

using namespace keti::common;

using grpc::Server;
// using grpc::ServerAsyncResponseWriter;
using grpc::ServerAsyncWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using osi3::Request;
using osi3::SensorView;
using osi3::SensorViewRPC;
using osi3::MountingPosition;
using osi3::HostVehicleData;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::RadarSensorView;
using osi3::GroundTruth;

using PhaseTable = std::unordered_map<int, std::vector<osi3::TrafficLight>>;
using TypeTable = std::unordered_map<int, PhaseTable>;
using ObjTable = std::map<std::pair<int, std::string>, osi3::MovingObject::VehicleClassification>;


class SensorDataOSIConverter {
  public:
    enum class VehicleGearStatus
    {
      NEUTRAL,
      DRIVE,
      REVERSE,
      PARKING
    };
    enum class VehicleMode
    {
      MANUAL,
      AUTO
    };
    struct VehicleGearAndModeInfo {
      VehicleGearStatus vehicle_gear_status;
      VehicleMode vehicle_mode;
    };

  public:

    SensorDataOSIConverter();
    ~SensorDataOSIConverter();
    void CameraSensorToOSI(const sensor_msgs::CompressedImageConstPtr& img_ros, CameraSensorView* camera_osi,
                           const MountingPosition& camera_mount_pose, const size_t& sensor_id);
    void LidarSensorToOSI(const sensor_msgs::PointCloud2ConstPtr& lidar_ros, LidarSensorView* lidar_osi, 
                          const MountingPosition& lidar_mount_pose, const size_t& num_of_rays, const size_t& sensor_id);
    void RadarSensorToOSI(const morai_msgs::RadarDetections& radardetections, RadarSensorView* radar_osi,
                          const MountingPosition& radar_mount_pose, const size_t& sensor_id);
    void RadarSensorToOSI(const radar_msgs::RadarScan& radarscans, RadarSensorView* radar_osi,
                          const geometry_msgs::PoseStamped::ConstPtr& radar_mount_pose, const size_t& sensor_id);

    void TrafficLightsToOSI(const morai_msgs::GetTrafficLightStatusConstPtr& traffic_light_ros, osi3::TrafficLight* traffic_ligth_gt, osi3::TrafficLight& traffic_ligth_osi);
    void TrafficLightIdMathching();
    void TrafficLightMoraiToOSIMatching();
    TypeTable GetMoraiToOSITrafficLightMatchingTable() { return morai_to_osi_matching_table_tl_; }
    void ObjectMoraiToOSIMatching();
    ObjTable GetMoraiToOSIObstacleMatchingTable() { return morai_to_osi_matching_table_obj_; }
    void EgoVehicleStateToOSI(const morai_msgs::EgoVehicleStatusConstPtr& ego_vehicle_state_ros, const sensor_msgs::Imu& imu_ros, const morai_msgs::GPSMessage& gps_morai, 
                             const autoware_msgs::VehicleStatus& autoware_vehicle_state_ros, const VehicleGearAndModeInfo& vehicle_gear_and_mode_info,
                             HostVehicleData* host_vehicle_osi);
    double GetEgoVehicleHeading () { return ego_vehicle_heading_; }
    double SetEgoVehicleHeading (double ege_vehicle_heading) { return ego_vehicle_heading_ = ege_vehicle_heading; }
    void EmptyObstacleToOSI(osi3::MovingObject* moving_obstacle_osi,
                            osi3::StationaryObject* stationary_obstacle_osi);
    void NPCObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, osi3::MovingObject* moving_obstacle_osi);
    void PedObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, osi3::MovingObject* moving_obstacle_osi);
    void StaticObstacleToOSI(const morai_msgs::ObjectStatus obstacle_ros, osi3::StationaryObject* stationary_obstacle_osi);
    void SetMoraiTMOffset(double (*morai_tm_offset)[2]) { morai_tm_offset_[0] = (*morai_tm_offset)[0], morai_tm_offset_[1] = (*morai_tm_offset)[1]; }
    double* GetMoraiTMOffset(){ return morai_tm_offset_; }
    void SetGeoCoordConv(const std::string* hdmap_path);
    CGeoCoordConv GetGeoCoordConv() { return geo_conv_; }
    CGeoCoordConv GetApolloOffsetCalGeoCoordConv() { return apollo_offset_cal_geo_conv_; }
    bool GetNeedRealTimeOffsetCal() { return need_real_time_offset_cal_; }
    void CalculateMoraiUtmOffset(const morai_msgs::EgoVehicleStatusConstPtr& ego_vehicle_state_ros, const morai_msgs::GPSMessage& gps_morai);

  private:
    std::unordered_map<std::string, std::vector<std::string>> id_table_;
    TypeTable morai_to_osi_matching_table_tl_;
    ObjTable morai_to_osi_matching_table_obj_;
    double ego_vehicle_heading_;
    std::string hdmap_path_;
    double morai_tm_offset_[2];
    CGeoCoordConv geo_conv_;
    CGeoCoordConv apollo_offset_cal_geo_conv_;
    bool need_real_time_offset_cal_ = false;

};