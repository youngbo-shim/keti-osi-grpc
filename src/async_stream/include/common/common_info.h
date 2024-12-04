#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

// math
#include "math/vec2d.h"
#include "math/vec3d.h"
#include "math/polygon2d.h"
#include "math/line_segment2d.h"
#include "math/aabox2d.h"
#include "math/aaboxkdtree2d.h"
#include "math/math_utils.h"

namespace keti {
namespace hdmap {


enum HDMapLayerType{ LANE, LINK, NODE, STOP, SPEEDBUMP };
// TODO(youngbo) : This part should consider the format of layers.
namespace Lane{
  enum Type{
    UNKNOWN = 0, SOLID_SINGLE_YELLOW=111, DASHED_SINGLE_YELLOW=112, SOLID_DOUBLE_YELLOW=121, DASHED_DOUBLE_YELLOW=122,
    SOLID_SINGLE_WHITE=211, DASHED_SINGLE_WHITE=212, SOLID_DOUBLE_WHITE=221, DASHED_DOUBLE_WHITE=222,
    SOLID_SINGLE_BLUE=311, DASHED_SINGLE_BLUE=312 ,OTHERS=999};
  enum Kind{
    CENTER_LINE = 501, UTURN_LINE = 502, NORMAL_LANE = 503, BUS_ONLY_LINE = 504, ROAD_EDGE_LINE = 505, NO_LANE_CHANGE_LINE = 506,
    NO_PARKING_LINE = 515, EXTENDED_BAY_LINE = 525, SAFE_ZONE = 531, BICYCLE_ROAD = 535, STOP_LINE = 530};
  enum Feature{
    R_LINKID, L_LINKID, LANETYPE, LANECODE, BARRIER, LNO, CODE,
    DATE, REMARK, HDUFID, HISTTYPE, HISTREMARK};
}

namespace Link{
  enum Feature{ LINKID, FROMNODE, TONODE, LENGTH, ROADTYPE, ROADNO, SPEED,
        LANE, CODE, GID, DATE,REMARK, ITS_LINKID, HDUFID, STOPID,LLID, RLID, NLIDS };

}

namespace Node{
	enum  Feature{ NODEID, NODETYPE, DATE, REMARK, ITS_NODEID,
        HDUFID, HISTTYPE, HISTREMARK, NLIDS, PLIDS};

}

namespace Stop{
  enum Feature{LINKID, CODE, DATE, REMARK, HDUFID};
}

namespace SpeedBump{
}

namespace SafetySign{

}

namespace SurfaceMark{

}

namespace ParkingLot{

}

namespace DrivewaySection{

}

namespace TrafficLight{

}

namespace PostPoint{

}

namespace VehicleProtectionSafety{

}

namespace HeightBarrier{

}

namespace InterSection{

}


template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

class LaneInfo;
class LinkInfo;
class NodeInfo;
class StopInfo;
class SpeedBumpInfo;
class SafetySignInfo;
class SurfaceMarkInfo;
class ParkingLotInfo;
class DrivewaySectionInfo;
class SubsidiarySectionInfo;
class TrafficLightInfo;
class PostPointInfo;
class VehicleProtectionSafetyInfo;
class HeightBarrierInfo;
class InterSectionInfo;

using LaneInfoPtr = std::shared_ptr<LaneInfo>;
using LinkInfoPtr = std::shared_ptr<LinkInfo>;
using NodeInfoPtr = std::shared_ptr<NodeInfo>;
using StopInfoPtr = std::shared_ptr<StopInfo>;
using SpeedBumpInfoPtr = std::shared_ptr<SpeedBumpInfo>;
using SafetySignInfoPtr = std::shared_ptr<SafetySignInfo>;
using SurfaceMarkInfoPtr = std::shared_ptr<SurfaceMarkInfo>;
using ParkingLotInfoPtr = std::shared_ptr<ParkingLotInfo>;
using DrivewaySectionInfoPtr = std::shared_ptr<DrivewaySectionInfo>;
using SubsidiarySectionInfoPtr = std::shared_ptr<SubsidiarySectionInfo>;
using TrafficLightInfoPtr = std::shared_ptr<TrafficLightInfo>;
using PostPointInfoPtr = std::shared_ptr<PostPointInfo>;
using VehicleProtectionSafetyInfoPtr = std::shared_ptr<VehicleProtectionSafetyInfo>;
using HeightBarrierInfoPtr = std::shared_ptr<HeightBarrierInfo>;
using InterSectionInfoPtr = std::shared_ptr<InterSectionInfo>;

using LaneTable = std::unordered_map<std::string, LaneInfoPtr>;
using LinkTable = std::unordered_map<std::string, LinkInfoPtr>;
using NodeTable = std::unordered_map<std::string, NodeInfoPtr>;
using StopTable = std::unordered_map<std::string, StopInfoPtr>;
using SpeedBumpTable = std::unordered_map<std::string, SpeedBumpInfoPtr>;
using SafetySignTable = std::unordered_map<std::string, SafetySignInfoPtr>;
using SurfaceMarkTable = std::unordered_map<std::string, SurfaceMarkInfoPtr>;
using ParkingLotTable = std::unordered_map<std::string, ParkingLotInfoPtr>;
using DrivewaySectionTable = std::unordered_map<std::string, DrivewaySectionInfoPtr>;
using SubsidiarySectionTable = std::unordered_map<std::string, SubsidiarySectionInfoPtr>;
using TrafficLightTable = std::unordered_map<std::string, TrafficLightInfoPtr>;
using PostPointTable = std::unordered_map<std::string, PostPointInfoPtr>;
using VehicleProtectionSafetyTable = std::unordered_map<std::string, VehicleProtectionSafetyInfoPtr>;
using HeightBarrierTable = std::unordered_map<std::string, HeightBarrierInfoPtr>;
using InterSectionTable = std::unordered_map<std::string, InterSectionInfoPtr>;

using LaneSegmentBox = ObjectWithAABox<LaneInfo, common::math::LineSegment2d>;
using LaneSegmentKDTree = common::math::AABoxKDTree2d<LaneSegmentBox>;
using LinkSegmentBox = ObjectWithAABox<LinkInfo, common::math::LineSegment2d>;
using LinkSegmentKDTree = common::math::AABoxKDTree2d<LinkSegmentBox>;
using NodeSegmentBox = ObjectWithAABox<NodeInfo, common::math::LineSegment2d>;
using NodeSegmentKDTree = common::math::AABoxKDTree2d<NodeSegmentBox>;
using TrafficLightSegmentBox = ObjectWithAABox<TrafficLightInfo, common::math::LineSegment2d>;
using TrafficLightSegmentKDTree = common::math::AABoxKDTree2d<TrafficLightSegmentBox>;
using StopSegmentBox = ObjectWithAABox<StopInfo, common::math::LineSegment2d>;
using StopSegmentKDTree = common::math::AABoxKDTree2d<StopSegmentBox>;
using InterSectionSegmentBox = ObjectWithAABox<InterSectionInfo, common::math::LineSegment2d>;
using InterSectionSegmentKDTree = common::math::AABoxKDTree2d<InterSectionSegmentBox>;
using SpeedBumpSegmentBox = ObjectWithAABox<SpeedBumpInfo, common::math::LineSegment2d>;
using SpeedBumpSegmentKDTree = common::math::AABoxKDTree2d<SpeedBumpSegmentBox>;
using DrivewaySectionSegmentBox = ObjectWithAABox<DrivewaySectionInfo, common::math::LineSegment2d>;
using DrivewaySectionSegmentKDTree = common::math::AABoxKDTree2d<DrivewaySectionSegmentBox>;
using SubsidiarySectionSegmentBox = ObjectWithAABox<SubsidiarySectionInfo, common::math::LineSegment2d>;
using SubsidiarySectionSegmentKDTree = common::math::AABoxKDTree2d<SubsidiarySectionSegmentBox>;
using ParkingLotSegmentBox = ObjectWithAABox<ParkingLotInfo, common::math::LineSegment2d>;
using ParkingLotSegmentKDTree = common::math::AABoxKDTree2d<ParkingLotSegmentBox>;
using SurfaceMarkSegmentBox = ObjectWithAABox<SurfaceMarkInfo, common::math::LineSegment2d>;
using SurfaceMarkSegmentKDTree = common::math::AABoxKDTree2d<SurfaceMarkSegmentBox>;
using SafetySignSegmentBox = ObjectWithAABox<SafetySignInfo, common::math::LineSegment2d>;
using SafetySignSegmentKDTree = common::math::AABoxKDTree2d<SafetySignSegmentBox>;
using VehicleProtectionSafetySegmentBox = ObjectWithAABox<VehicleProtectionSafetyInfo, common::math::LineSegment2d>;
using VehicleProtectionSafetySegmentKDTree = common::math::AABoxKDTree2d<VehicleProtectionSafetySegmentBox>;
using PostPointSegmentBox = ObjectWithAABox<PostPointInfo, common::math::LineSegment2d>;
using PostPointSegmentKDTree = common::math::AABoxKDTree2d<PostPointSegmentBox>;
using HeightBarrierSegmentBox = ObjectWithAABox<HeightBarrierInfo, common::math::LineSegment2d>;
using HeightBarrierSegmentKDTree = common::math::AABoxKDTree2d<HeightBarrierSegmentBox>;

// Overlaps data : (ID, start_s, end_s)
using OverlapsTable = std::unordered_map<std::string, std::pair<double, double>>;

/*
* --- If the vehicle does not need to change lane, then change_lane_type ==
*     FORWARD;
* --- If the vehicle need to change to left lane according to routing, then
*     change_lane_type_ == LEFT;
* --- If the vehicle need to change to right lane according to routing, then
*     change_lane_type_ == RIGHT;
*/
enum ChangeLaneType : int {
  FORWARD = 0,
  LEFT = 1,
  RIGHT = 2
};

enum LaneTurn {
  NO_TURN = 1,
  LEFT_TURN = 2,
  RIGHT_TURN = 3,
  U_TURN = 4
};

enum LinkType {
  NORMAL = 6,
  INTERSECTION = 1,
  SPLIT = 13,
  MERGE = 14
};

enum NodeType {
  FLAT_INTERSECTION = 1,
  INTERCHANGE = 2,
  TUNNEL = 3,
  BRIDGE = 4,
  UNDERPASS = 5,
  OVERPASS = 6,
  ROAD_ODER_CHANGE = 7,
  TOLLGATE_START_END = 8,
  TOLLGATE = 9,
  ROUNDABOUT = 10,
  OTHERS_NODE_TYPE = 11
};

enum TrafficLightType {
  HORIZONTAL_TRICOLOR = 1,              // 차량횡형-삼색등
  HORIZONTAL_FOURCOLORA = 2,            // 차량횡형-사색등A
  HORIZONTAL_FOURCOLORB = 3,            // 차량횡형-사색등B
  HORIZONTAL_ARROW_TRICOLOR = 4,        // 차량횡형-화살표삼색등
  VERTICAL_TRICOLOR = 5,                // 차량종형-삼색등
  VERTICAL_ARROW_TRICOLOR = 6,          // 차량종형-화살표삼색등
  VERTICAL_FOURCOLOR = 7,               // 차량종형-사색등
  BUS_TRICOLOR = 8,                     // 버스삼색등
  VARIABLE_TRAFFIC_LANE_CONTROL = 9,    // 가변형 가변등
  VARIABLE_ALARM = 10,                  // 경보형 가변등
  PEDESTRIAN = 11,                      // 보행등
  VERTICAL_BICYCLE_TRICOLOR = 12,       // 자전거종형-삼색등
  VERTICAL_BICYCLE_TWOCOLOR = 13,       // 자전거종형-이색등
  VERTICCAL_AUXILIARY_TRICOLOR = 14,    // 차량보조등-종형삼색등
  VERTICCAL_AUXILIARY_FOURCOLOR = 15,   // 차량보조등-종형사색등
  OTHERS_TRAFFIC_LIGHT_TYPE = 99        // 기타 신호등 유형
};

} // namespace hdmap
} // namespace keti