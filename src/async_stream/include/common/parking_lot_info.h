#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class ParkingLotInfo
 *
 * @brief Information of parking lot in High-precision map.
 */
class ParkingLotInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const std::string &section_id() const { return section_id_; }
  const unsigned int &type() const { return type_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const common::math::Polygon2d &polygon() const { return polygon_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return mean_point_based_segments_; }
  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_section_id(std::string section_id) { section_id_ = section_id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }
  void set_polygon_from_points(std::vector<common::math::Vec2d> points) { polygon_ = common::math::Polygon2d(points); }
  void InitSegmentWithMeanPoint(){ mean_point_based_segments_.emplace_back(polygon_.GetMeanPoint(), polygon_.GetMeanPoint()); }
private:
  // Attributes
  std::string id_; // 고유 id
  std::string section_id_; // 주차장을 포함하는 부속 구간(A4_SUBSIDIARYSECTION)) id 
  unsigned int type_;  // 1: 일반 주차장, 2: 화물차 전용 주차장, ... 9: 기타 주차장
  std::vector<common::math::Vec3d> points_3d_;
  common::math::Polygon2d polygon_;
  std::vector<common::math::LineSegment2d> mean_point_based_segments_; // for kdtree construction
};

} // namespace hdmap
} // namespace keti