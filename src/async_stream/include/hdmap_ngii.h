/**
 * @file hdmap_ngii.h
 *
 * @brief Declaration of the class HDMapNGII.
 */

#include "hdmap_common.h"

namespace keti{
namespace hdmap{

// TODO(youngbo) : This part should consider the format of layers.
namespace Lane{
  namespace NGII{
    enum Feature{
      ID, AdminCode, Type, Kind, R_linkID, L_linkID, Maker,
      UpdateDate, Version, Remark, HistType, HistRemark};
    enum Type{
      UNKNOWN = 0, SOLID_SINGLE_YELLOW=111, DASHED_SINGLE_YELLOW=112, SOLID_DOUBLE_YELLOW=121, DASHED_DOUBLE_YELLOW=122,
      SOLID_SINGLE_WHITE=211, DASHED_SINGLE_WHITE=212, SOLID_DOUBLE_WHITE=221, DASHED_DOUBLE_WHITE=222, OTHERS=999};
  }
}

namespace Link{
  namespace NGII{
    enum Feature{ ID, AdminCode, RoadRank, RoadType, RoadNo, LinkType, MaxSpeed, LaneNo, R_LinkID, L_LinkID, FromNodeID, ToNodeID, SectionID, Length,
                  ITSLinkID, Marker, UpdateDate, Version, Remark, HistType, HistRemark, TurnType};
  }
}

namespace Node{
  namespace NGII{
    enum Feature{ ID, AdminCode, NodeType, ITSNodeID, Maker, UpdateDate, Version, Remark, HistType, HistRemark };
  }
}

namespace SpeedBump{
  namespace NGII{
    enum Feature{ ID, AdminCode, Type, LinkID, Ref_Lane, Marker, UpdateDate, Version, Remark, HistType, HistRemark };
  }
}

namespace SafetySign{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, SubType, LinkID, Ref_Lane, Maker, Post_ID };
  }
}

namespace SurfaceMark{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, Kind, LinkID };
  }
}

namespace ParkingLot{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, SectionID };
  }
}

namespace DrivewaySection{
  namespace NGII {
    enum Feature { ID, AdminCode, Kind, RoadType };
  }
}

namespace TrafficLight{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, LinkID, Ref_Lane, Maker, Post_ID };
  }
}


namespace PostPoint{
  namespace NGII {
    enum Feature { ID, AdminCode, Type };
  }
}

namespace VehicleProtectionSafety{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, IsCentral, LowHigh, Ref_FID };
  }
}

namespace HeightBarrier{
  namespace NGII {
    enum Feature { ID, AdminCode, Type, LinkID, Ref_Lane };
  }
}

namespace InterSection{
  namespace NGII {
    enum Feature { ID };
  }
}

namespace SubsidiarySection{
  namespace NGII {
    enum Feature { ID, AdminCode, SubType, Name, Direction, GasStation, LgpStation, EvCharger,Toilet,
                   Maker, UpdateDate, Version, Remark, HistType, HistRemark };
  }
}

class HDMapNGII : public HDMapLoader{
public:
    
    HDMapNGII(){};
    HDMapNGII(std::string& hdmap_from, std::string& data_path,
              std::string& lane_filename, std::string& link_filename, std::string& node_filename,
              std::string speedbump_filename="", std::string safetysign_filename="",
              std::string surfacemark_filename="", std::string parkinglot_filename="",
              std::string drivewaysection_filename="", std::string trafficlight_filename="",
              std::string postpoint_filename="", std::string vehicleprotectionsafety_filename="",
              std::string heightbarrier_filename="", std::string intersection_filename="", std::string subsidiarysection_filename="");
    
    void UpdateOverlaps() final;
    bool LoadMapFromFile(std::string& hdmap_from, std::string& data_path,
                        std::string& lane_filename, std::string& link_filename, std::string& node_filename,
                        std::string speedbump_filename="", std::string safetysign_filename="",
                        std::string surfacemark_filename="", std::string parkinglot_filename="",
                        std::string drivewaysection_filename="", std::string trafficlight_filename="",
                        std::string postpoint_filename="", std::string vehicleprotectionsafety_filename="",
                        std::string heightbarrier_filename="", std::string intersection_filename="", std::string subsidiarysection_filename="") final;

private:
    bool LoadNGIIMapLaneFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapLINKFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapNodeFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapSpeedBumpFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapSafetySignFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapSurfaceMarkFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapParkingLotFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapDriveWaySectionFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);                                                  
    bool LoadNGIIMapTrafficLightFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);                                                  
    bool LoadNGIIMapPostPointFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);
    bool LoadNGIIMapVehicleProtectionSafetyFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);    
    bool LoadNGIIMapHeightBarrierFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);           
    bool LoadNGIIMapInterSectionFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);  
    bool LoadNGIIMapSubsidiarySectionFromFile(const std::string &shp_filename,
                         const std::string &dbf_filename);                                                                        
};

} // namespace hdmap
} // namespace keti
