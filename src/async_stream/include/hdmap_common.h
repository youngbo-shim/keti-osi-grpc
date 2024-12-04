#pragma once

// ros
#include "geometry_msgs/Point.h"

// libshp
#include "libshp/shapefil.h"

#include "common/common_info.h"
#include "common/lane_info.h"
#include "common/link_info.h"
#include "common/node_info.h"
#include "common/speedbump_info.h"
#include "common/stop_info.h"
#include "common/safetysign_info.h"
#include "common/surfacemark_info.h"
#include "common/parking_lot_info.h"
#include "common/driveway_section_info.h"
#include "common/trafficlight_info.h"
#include "common/postpoint_info.h"
#include "common/height_barrier_info.h"
#include "common/vehicle_protection_safety_info.h"
#include "common/intersection_info.h"
#include "common/subsidiary_section_info.h"

namespace keti{
namespace hdmap{

/**
 * @class HDMapLoader
 *
 * @brief High-precision map loader implement.
 */
class HDMapLoader{
public:	

  HDMapLoader(){};

  virtual bool LoadMapFromFile(std::string& hdmap_from, std::string& data_path,
                              std::string& lane_filename, std::string& link_filename, std::string& node_filename,
                              std::string speedbump_filename="", std::string safetysign_filename="", std::string surfacemark_filename="",
                              std::string parkinglot_filename="", std::string drivewaysection_filename="", std::string trafficlight_filename="",
                              std::string postpoint_filename="", std::string vehicleprotectionsafety_filename="",
                              std::string heightbarrier_filename="", std::string intersection_filename="", std::string subsidiarysection_filename="") = 0;

  void Clear();
  bool IsValid();
  
  LaneInfoPtr GetLaneById(const std::string& id) const;
  LinkInfoPtr GetLinkById(const std::string& id) const;
  NodeInfoPtr GetNodeById(const std::string& id) const;
  TrafficLightInfoPtr GetTrafficLightById(const std::string& id) const;
  InterSectionInfoPtr GetInterSectionById(const std::string& id) const;
  SurfaceMarkInfoPtr GetSurfaceMarkById(const std::string &id) const;
  DrivewaySectionInfoPtr GetDrivewaySectionById(const std::string& id) const;
  SubsidiarySectionInfoPtr GetSubsidiarySectionById(const std::string& id) const;
  ParkingLotInfoPtr GetParkingLotById(const std::string& id) const;
  SpeedBumpInfoPtr GetSpeedBumpById(const std::string& id) const;
  SafetySignInfoPtr GetSafetySignById(const std::string& id) const;
  VehicleProtectionSafetyInfoPtr GetVehicleProtectionSafetyById(const std::string& id) const;
  PostPointInfoPtr GetPostPointById(const std::string& id) const;
  HeightBarrierInfoPtr GetHeightBarrierById(const std::string& id) const;
  
  template <class Table, class BoxTable, class KDTree>
  void BuildSegmentKDTree(const Table& table, const common::math::AABoxKDTreeParams& params,
                            BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  void InsertLinkNodeInfo();

  void InsertTrafficLightsIntoSurfaceMark();
  void InsertIntersectionIdIntoTrafficLights();

  void InsertHeadingIntoTrafficLights();
  void InsertHeadingIntoSafetySigns();

  void BuildLinkedTable();

  void BuildLaneSegmentKDTree();
  void BuildLinkSegmentKDTree();
  void BuildNodeSegmentKDTree();
  void BuildTrafficLightSegmentKDTree();
  void BuildIntersectionKDTree();
  void BuildDrivewaySectionKDTree();
  void BuildSubsidiarySectionKDTree();
  void BuildSurfaceMarkKDTree();
  void BuildParkingLotKDTree();
  void BuildSpeedBumpKDTree();
  void BuildSafetySignKDTree();
  void BuildVehicleProtectionSafetyKDTree();
  void BuildPostPointKDTree();
  void BuildHeightBarrierKDTree();

  LaneTable GetLaneTable() { return lane_table_; }
  LinkTable GetLinkTable() { return link_table_; }
  NodeTable GetNodeTable() { return node_table_; }
  StopTable GetStopTable() { return stop_table_; }
  SpeedBumpTable GetSpeedBumpTable() { return speedbump_table_; }
  SurfaceMarkTable GetSurfaceMarkTable() { return surfacemark_table_; }
  ParkingLotTable GetParkingLotTable() { return parkinglot_table_; }
  DrivewaySectionTable GetDrivewaySectionTable() { return drivewaysection_table_; }
  SubsidiarySectionTable GetSubsidiarySectionTable() { return subsidiarysection_table_; }
  SafetySignTable GetSafetySignTable() { return safetysign_table_;}
  TrafficLightTable GetTrafficLightTable() { return trafficlight_table_; }
  PostPointTable GetPostPointTable() { return postpoint_table_; }
  VehicleProtectionSafetyTable GetVehicleProtectionSafetyTable() { return vehicleprotectionsafety_table_; }
  HeightBarrierTable GetHeightBarrierTable() { return heightbarrier_table_; }
  InterSectionTable GetInterSectionTable() { return intersection_table_; }

  int GetNearestLink(const geometry_msgs::Point& point,
                              LinkInfoPtr* nearest_link, double* nearest_s,
                              double* nearest_l) const;

  int GetNearestLink(const common::math::Vec2d& point,
                              LinkInfoPtr* nearest_link, double* nearest_s,
                              double* nearest_l) const;

  int GetNearestLinkWithHeading(const geometry_msgs::Point& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LinkInfoPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;

  int GetNearestLinkWithHeading(const common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LinkInfoPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;

  int GetLinksWithHeading(const geometry_msgs::Point& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LinkInfoPtr>* lanes) const;  

  int GetLinksWithHeading(const common::math::Vec2d& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LinkInfoPtr>* lanes) const;  

  int GetLanes(const geometry_msgs::Point& point, double distance,
              std::vector<LaneInfoPtr>* lanes) const;    
  int GetLinks(const geometry_msgs::Point& point, double distance,
              std::vector<LinkInfoPtr>* links) const;   
  int GetNodes(const geometry_msgs::Point& point, double distance,
              std::vector<NodeInfoPtr>* nodes) const;
  int GetTrafficLights(const geometry_msgs::Point& point, double distance,
                      std::vector<TrafficLightInfoPtr>* trafficlights) const;
  int GetSpeedBumps(const geometry_msgs::Point& point, double distance,
                      std::vector<SpeedBumpInfoPtr>* speedbumps) const;
  int GetDrivewaySections(const geometry_msgs::Point& point, double distance,
                      std::vector<DrivewaySectionInfoPtr>* drivewaysections) const;
  int GetParkingLots(const geometry_msgs::Point& point, double distance,
                      std::vector<ParkingLotInfoPtr>* parkinglots) const;
  int GetSurfaceMarks(const geometry_msgs::Point& point, double distance,
                      std::vector<SurfaceMarkInfoPtr>* surfacemarks) const;
  int GetSafetySigns(const geometry_msgs::Point& point, double distance,
               std::vector<SafetySignInfoPtr>* safetysigns) const;
  int GetVehicleProtectionSafetys(const geometry_msgs::Point& point, double distance,
               std::vector<VehicleProtectionSafetyInfoPtr>* vehicleprotectionsafetys) const;
  int GetPostPoints(const geometry_msgs::Point& point, double distance,
               std::vector<PostPointInfoPtr>* postpoints) const;
  int GetHeightBarriers(const geometry_msgs::Point& point, double distance,
               std::vector<HeightBarrierInfoPtr>* heightbarriers) const;
  
  int GetLanes(const common::math::Vec2d& point, double distance,
              std::vector<LaneInfoPtr>* lanes) const;
  int GetLinks(const common::math::Vec2d& point, double distance,
              std::vector<LinkInfoPtr>* links) const;
  int GetNodes(const common::math::Vec2d& point, double distance,
              std::vector<NodeInfoPtr>* nodes) const;
  int GetTrafficLights(const common::math::Vec2d &point, double distance,
                      std::vector<TrafficLightInfoPtr>* trafficlights) const;  
  int GetInterSections(const common::math::Vec2d &point, double distance,
                      std::vector<InterSectionInfoPtr>* intersections) const;  
  int GetSubsidiarySections(const common::math::Vec2d &point, double distance,
                           std::vector<SubsidiarySectionInfoPtr>* subsidiarysections) const;    
  int GetSpeedBumps(const common::math::Vec2d &point, double distance,
                      std::vector<SpeedBumpInfoPtr>* speedbumps) const;
  int GetDrivewaySections(const common::math::Vec2d &point, double distance,
                      std::vector<DrivewaySectionInfoPtr>* drivewaysections) const;
  int GetParkingLots(const common::math::Vec2d &point, double distance,
                      std::vector<ParkingLotInfoPtr>* parkinglots) const;
  int GetSurfaceMarks(const common::math::Vec2d &point, double distance,
                      std::vector<SurfaceMarkInfoPtr>* surfacemarks) const;
  int GetSafetySigns(const common::math::Vec2d &point, double distance,
               std::vector<SafetySignInfoPtr>* safetysigns) const;
  int GetVehicleProtectionSafetys(const common::math::Vec2d &point, double distance,
               std::vector<VehicleProtectionSafetyInfoPtr>* vehicleprotectionsafetys) const;
  int GetPostPoints(const common::math::Vec2d &point, double distance,
               std::vector<PostPointInfoPtr>* postpoints) const;
  int GetHeightBarriers(const common::math::Vec2d &point, double distance,
               std::vector<HeightBarrierInfoPtr>* heightbarriers) const;


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
  int GetInterSections(const geometry_msgs::Point &point, double distance,
                       std::vector<InterSectionInfoPtr> *intersections) const;
  int GetSubsidiarySections(const SubsidiarySectionTable& subsidiarysection_table, std::vector<SubsidiarySectionInfoPtr>* subsidiarysections) const;
  int GetSubsidiarySections(const geometry_msgs::Point &point, double distance,
                            std::vector<SubsidiarySectionInfoPtr> *subsidiarysections) const;
  const std::unique_ptr<LaneSegmentKDTree> &LaneKDTree() const { return lane_segment_kdtree_; }	
  const std::unique_ptr<LinkSegmentKDTree> &LinkKDTree() const { return link_segment_kdtree_; }	
  const std::unique_ptr<NodeSegmentKDTree> &NodeKDTree() const { return node_kdtree_; }
  const std::unique_ptr<TrafficLightSegmentKDTree> &TrafficLightKDTree() const { return trafficlight_kdtree_; }

  const double &x_offset() const { return x_offset_; }
  const double &y_offset() const { return y_offset_; }

protected:
  virtual void UpdateOverlaps() = 0;
  void UpdateSpeedBumpOverlapFromLink(const SpeedBumpInfo& speed_bump);
  void UpdateSurfaceMarkOverlapFromLink(const SurfaceMarkInfo& surfacemark);
  void UpdateTrafficLightOverlapFromLink(const TrafficLightInfo &traffic_light);
  void UpdateIntersectionOverlapFromLink(LinkInfo &link);
  void UpdateUTurnStopOverlapFromLink(LinkInfo &link);
  void UpdateCreepStopOverlapFromLink(LinkInfo &link);
  void UpdateDriveWaySectionOverlapFromLink(const DrivewaySectionInfo &drivewaysection);

protected:
  std::string hdmap_from_;
  double x_offset_, y_offset_;

  LaneTable lane_table_;
  LinkTable link_table_;
  NodeTable node_table_;
  StopTable stop_table_;
  VehicleProtectionSafetyTable vehicleprotectionsafety_table_;
  HeightBarrierTable heightbarrier_table_;
  SafetySignTable safetysign_table_;
  TrafficLightTable trafficlight_table_;
  PostPointTable postpoint_table_;
  SpeedBumpTable speedbump_table_;
  SurfaceMarkTable surfacemark_table_;
  ParkingLotTable parkinglot_table_;
  DrivewaySectionTable drivewaysection_table_;
  SubsidiarySectionTable subsidiarysection_table_;
  InterSectionTable intersection_table_;

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  std::vector<LinkSegmentBox> link_segment_boxes_;
  std::unique_ptr<LinkSegmentKDTree> link_segment_kdtree_;

  std::vector<NodeSegmentBox> node_boxes_;
  std::unique_ptr<NodeSegmentKDTree> node_kdtree_;

  std::vector<TrafficLightSegmentBox> trafficlight_boxes_;
  std::unique_ptr<TrafficLightSegmentKDTree> trafficlight_kdtree_;

  std::vector<InterSectionSegmentBox> intersection_boxes_;
  std::unique_ptr<InterSectionSegmentKDTree> intersection_kdtree_;

  std::vector<SubsidiarySectionSegmentBox> subsidiarysection_boxes_;
  std::unique_ptr<SubsidiarySectionSegmentKDTree> subsidiarysection_kdtree_;

  std::vector<SpeedBumpSegmentBox> speedbump_boxes_;
  std::unique_ptr<SpeedBumpSegmentKDTree> speedbump_kdtree_;

  std::vector<DrivewaySectionSegmentBox> drivewaysection_boxes_;
  std::unique_ptr<DrivewaySectionSegmentKDTree> drivewaysection_kdtree_;

  std::vector<ParkingLotSegmentBox> parkinglot_boxes_;
  std::unique_ptr<ParkingLotSegmentKDTree> parkinglot_kdtree_;

  std::vector<SurfaceMarkSegmentBox> surfacemark_boxes_;
  std::unique_ptr<SurfaceMarkSegmentKDTree> surfacemark_kdtree_;

  std::vector<SafetySignSegmentBox> safetysign_boxes_;
  std::unique_ptr<SafetySignSegmentKDTree> safetysign_kdtree_;

  std::vector<VehicleProtectionSafetySegmentBox> vehicleprotectionsafety_boxes_;
  std::unique_ptr<VehicleProtectionSafetySegmentKDTree> vehicleprotectionsafety_kdtree_;

  std::vector<PostPointSegmentBox> postpoint_boxes_;
  std::unique_ptr<PostPointSegmentKDTree> postpoint_kdtree_;

  std::vector<HeightBarrierSegmentBox> heightbarrier_boxes_;
  std::unique_ptr<HeightBarrierSegmentKDTree> heightbarrier_kdtree_;

private:
  // The static allows access without an object. 
  template <class KDTree>
  static int SearchObjects(const common::math::Vec2d& center, const double radius,
                    const KDTree& kdtree,
                    std::vector<std::string>* const results);
};

} // namespace hdmap
} // namespace keti
