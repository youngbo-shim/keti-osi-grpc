#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class SubsidiarySectionInfo
 *
 * @brief Information of sub-sidiary section in High-precision map.
 */
class SubsidiarySectionInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }  
  const unsigned int &sub_type() const { return sub_type_; }
  const unsigned int &direction() const { return direction_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const common::math::Polygon2d &polygon() const { return polygon_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return mean_point_based_segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_sub_type(unsigned int sub_type) { sub_type_ = sub_type; }
  void set_direction(unsigned int direction) { direction_ = direction; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }
  void set_polygon_from_points(std::vector<common::math::Vec2d> points) { polygon_ = common::math::Polygon2d(points); }
  void InitSegmentWithMeanPoint(){ mean_point_based_segments_.emplace_back(polygon_.GetMeanPoint(), polygon_.GetMeanPoint()); }

private:
  // Attributes
  std::string id_; // 고유 id
  unsigned int sub_type_;  // 1: 휴게소, 2: 졸음쉼터, 3: 보도, 4: 자전거도로, 9: 기타부속구간
  unsigned int direction_; // 방향 - 1: 상행, 2: 하행, 3: 양방향
  std::vector<common::math::Vec3d> points_3d_;
  common::math::Polygon2d polygon_;
  std::vector<common::math::LineSegment2d> mean_point_based_segments_; // for kdtree construction
};

} // namespace hdmap
} // namespace keti