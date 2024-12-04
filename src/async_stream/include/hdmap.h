/**
 * @file hdmap.h
 *
 * @brief Declaration of the class HDMap.
 */
#pragma once

#include "hdmap_common.h"
#include "geometry_msgs/Point.h"

namespace keti{
namespace hdmap{

class HDMap{
public:

  HDMap(){};
  HDMap(std::string& hdmap_from, std::string& data_path,
        std::string& lane_filename, std::string& link_filename, std::string& node_filename,
        std::string speedbump_filename="", std::string safetysign_filename="", std::string surfacemark_filename="",
        std::string parkinglot_filename="", std::string drivewaysection_filename="", std::string trafficlight_filename="",
        std::string postpoint_filename="", std::string vehicleprotectionsafety_filename="",
        std::string heightbarrier_filename="", std::string intersection_filename="", std::string subsidiarysection_filename="");

  /**
   * @brief initialize hdmap type instance (like NGII, NAVER, KAIST, etc)
   * @param map_filename path of map data file(lane, link, node, etc)
   * @return 1:success, otherwise failed
  */
  bool ResetLoader(std::string& hdmap_from, std::string& data_path,
                   std::string& lane_filename, std::string& link_filename, std::string& node_filename,
                   std::string speedbump_filename="", std::string safetysign_filename="", std::string surfacemark_filename="",
                   std::string parkinglot_filename="", std::string drivewaysection_filename="", std::string trafficlight_filename="",
                   std::string postpoint_filename="", std::string vehicleprotectionsafety_filename="",
                   std::string heightbarrier_filename="", std::string intersection_filename="", std::string subsidiarysection_filename="");

  LaneInfoPtr GetLaneById(const std::string& id) const;
  LinkInfoPtr GetLinkById(const std::string& id) const;
  NodeInfoPtr GetNodeById(const std::string& id) const;
  TrafficLightInfoPtr GetTrafficLightById(const std::string& id) const;
  SurfaceMarkInfoPtr GetSurfaceMarkById(const std::string& id) const;
  DrivewaySectionInfoPtr GetDrivewaySectionById(const std::string& id) const;
  SubsidiarySectionInfoPtr GetSubsidiarySectionById(const std::string& id) const;
  ParkingLotInfoPtr GetParkingLotById(const std::string& id) const;
  InterSectionInfoPtr GetInterSectionById(const std::string& id) const;
  SpeedBumpInfoPtr GetSpeedBumpById(const std::string& id) const;
  SafetySignInfoPtr GetSafetySignById(const std::string& id) const;
  VehicleProtectionSafetyInfoPtr GetVehicleProtectionSafetyById(const std::string& id) const;
  PostPointInfoPtr GetPostPointById(const std::string& id) const;
  HeightBarrierInfoPtr GetHeightBarrierById(const std::string& id) const;
  
  /**
   * @brief get nearest link from target point,
   * @param point the target point
   * @param nearest_link the nearest link that match search conditions
   * @param nearest_s the offset from link start point along link center line
   * @param nearest_l the lateral offset from link center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLink(const geometry_msgs::Point& point,
                    LinkInfoPtr* nearest_link, double* nearest_s,
                    double* nearest_l) const;

  /**
   * @brief get the nearest link within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_link the nearest link that match search conditions
   * @param nearest_s the offset from link start point along link center line
   * @param nearest_l the lateral offset from link center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLinkWithHeading(const geometry_msgs::Point& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LinkInfoPtr* nearest_link,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all links within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_link all links that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLinksWithHeading(const geometry_msgs::Point& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LinkInfoPtr>* links) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const geometry_msgs::Point& point, double distance,
               std::vector<LaneInfoPtr>* lanes) const;

  int GetLanes(const common::math::Vec2d &point, double distance,
               std::vector<LaneInfoPtr> *lanes) const;

  /**
   * @brief get all links in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param links store all links in target range
   * @return 0:success, otherwise failed
   */
  int GetLinks(const geometry_msgs::Point& point, double distance,
               std::vector<LinkInfoPtr>* links) const;

  /**
   * @brief get all nodes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param nodes store all nodes in target range
   * @return 0:success, otherwise failed
   */
  int GetNodes(const geometry_msgs::Point& point, double distance,
               std::vector<NodeInfoPtr>* nodes) const;

  /**
   * @brief get all traffic lights in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param trafficlights store all traffic lights in target range
   * @return 0:success, otherwise failed
   */
  int GetTrafficLights(const geometry_msgs::Point& point, double distance,
                      std::vector<TrafficLightInfoPtr>* trafficlights) const;

  /**
   * @brief get all traffic intersection in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param intersections store all intersections in target range
   * @return 0:success, otherwise failed
   */
  int GetInterSections(const geometry_msgs::Point& point, double distance,
                      std::vector<InterSectionInfoPtr>* intersections) const;

  /**
   * @brief get all subsidiarysection in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param subsidiarysections store all subsidiarysection in target range
   * @return 0:success, otherwise failed
   */
  int GetSubsidiarySections(const geometry_msgs::Point& point, double distance,
                            std::vector<SubsidiarySectionInfoPtr>* subsidiarysections) const;  

  /**
   * @brief get all speedbumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speedbumps store all speenbumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const geometry_msgs::Point& point, double distance,
                      std::vector<SpeedBumpInfoPtr>* speedbumps) const;

  /**
   * @brief get all drivewaysections in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param drivewaysections store all drivewaysections in target range
   * @return 0:success, otherwise failed
   */
  int GetDrivewaySections(const geometry_msgs::Point& point, double distance,
                      std::vector<DrivewaySectionInfoPtr>* drivewaysections) const;

  /**
   * @brief get all parkinglots in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parkinglots store all parkinglots in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingLots(const geometry_msgs::Point& point, double distance,
                      std::vector<ParkingLotInfoPtr>* parkinglots) const;

  /**
   * @brief get all surfacemarks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param surfacemarks store all surfacemarks in target range
   * @return 0:success, otherwise failed
   */
  int GetSurfaceMarks(const geometry_msgs::Point& point, double distance,
                      std::vector<SurfaceMarkInfoPtr>* surfacemarks) const;

  /**
   * @brief get all safetysigns in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param safetysigns store all safetysigns in target range
   * @return 0:success, otherwise failed
   */
  int GetSafetySigns(const geometry_msgs::Point& point, double distance,
               std::vector<SafetySignInfoPtr>* safetysigns) const;

  /**
   * @brief get all vehicleprotectionsafetys in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param vehicleprotectionsafetys store all vehicleprotectionsafetys in target range
   * @return 0:success, otherwise failed
   */
  int GetVehicleProtectionSafetys(const geometry_msgs::Point& point, double distance,
               std::vector<VehicleProtectionSafetyInfoPtr>* vehicleprotectionsafetys) const;

  /**
   * @brief get all postpoints in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param postpoints store all postpoints in target range
   * @return 0:success, otherwise failed
   */
  int GetPostPoints(const geometry_msgs::Point& point, double distance,
               std::vector<PostPointInfoPtr>* postpoints) const;

  /**
   * @brief get all heightbarriers in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param heightbarriers store all heightbarriers in target range
   * @return 0:success, otherwise failed
   */
  int GetHeightBarriers(const geometry_msgs::Point& point, double distance,
               std::vector<HeightBarrierInfoPtr>* heightbarriers) const;



  bool IsValid() { return loader_ptr_->IsValid(); }

  const double &x_offset() const { return loader_ptr_->x_offset(); }
  const double &y_offset() const { return loader_ptr_->y_offset(); }
  
  SpeedBumpTable GetSpeedBumpTable() { return loader_ptr_->GetSpeedBumpTable(); }
  SurfaceMarkTable GetSurfaceMarkTable() { return loader_ptr_->GetSurfaceMarkTable(); }
  ParkingLotTable GetParkingLotTable() { return loader_ptr_->GetParkingLotTable(); }
  DrivewaySectionTable GetDrivewaySectionTable() { return loader_ptr_->GetDrivewaySectionTable(); }
  SubsidiarySectionTable GetSubsidiarySectionTable() { return loader_ptr_->GetSubsidiarySectionTable(); }
  SafetySignTable GetSafetySignTable() { return loader_ptr_->GetSafetySignTable();}
  TrafficLightTable GetTrafficLightTable() { return loader_ptr_->GetTrafficLightTable(); }
  PostPointTable GetPostPointTable() { return loader_ptr_->GetPostPointTable(); }
  VehicleProtectionSafetyTable GetVehicleProtectionSafetyTable() { return loader_ptr_->GetVehicleProtectionSafetyTable(); }
  HeightBarrierTable GetHeightBarrierTable() { return loader_ptr_->GetHeightBarrierTable(); }
  InterSectionTable GetInterSectionTable() { return loader_ptr_->GetInterSectionTable(); }  

  int GetSpeedBumps(const SpeedBumpTable& speedbump_table, std::vector<SpeedBumpInfoPtr>* speedbumps) const;
  int GetSurfaceMarks(const SurfaceMarkTable& surfacemark_table, std::vector<SurfaceMarkInfoPtr>* surfacemarks) const;
  int GetParkingLots(const ParkingLotTable& parkinglot_table, std::vector<ParkingLotInfoPtr>* parkinglots) const;
  int GetDrivewaySections(const DrivewaySectionTable& drivewaysection_table, std::vector<DrivewaySectionInfoPtr>* drivewaysections) const;
  int GetSafetySigns(const SafetySignTable& safetysign_table, std::vector<SafetySignInfoPtr>* safetysigns) const;
  int GetTrafficLights(const TrafficLightTable& trafficlight_table, std::vector<TrafficLightInfoPtr>* trafficlights) const;
  int GetPostPoints(const PostPointTable& postpoint_table, std::vector<PostPointInfoPtr>* postpoints) const;
  int GetVehicleProtectionSafetys(const VehicleProtectionSafetyTable& vehicleprotectionsafety_table, std::vector<VehicleProtectionSafetyInfoPtr>* vehicleprotectionsafetys) const;
  int GetHeightBarriers(const HeightBarrierTable& heightbarrier_table, std::vector<HeightBarrierInfoPtr>* heightbarriers) const;
  int GetInterSections(const InterSectionTable& intersection_table, std::vector<InterSectionInfoPtr>* intersections) const;
  int GetSubsidiarySections(const SubsidiarySectionTable& subsidiarysection_table, std::vector<SubsidiarySectionInfoPtr>* subsidiarysections) const;
  
private:
  std::shared_ptr<HDMapLoader> loader_ptr_;
};

} // namespace hdmap
} // namespace keti
