#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class DrivewaySectionInfo
 *
 * @brief Information of driveway section in High-precision map.
 */
class DrivewaySectionInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const unsigned int &kind() const { return kind_; }
  const unsigned int &road_type() const { return road_type_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const common::math::Polygon2d &polygon() const { return polygon_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return mean_point_based_segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_kind(unsigned int kind) { kind_ = kind; }
  void set_road_type(unsigned int road_type) { road_type_ = road_type; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }
  void set_polygon_from_points(std::vector<common::math::Vec2d> points) { polygon_ = common::math::Polygon2d(points); }
  void InitSegmentWithMeanPoint(){ mean_point_based_segments_.emplace_back(polygon_.GetMeanPoint(), polygon_.GetMeanPoint()); }

private:
  // Attributes
  std::string id_; // 고유 id
  unsigned int kind_; // 차도 구간 유형 - 1: 주행 구간, 7: 자율주행 금지 구간
  unsigned int road_type_;  // 1: 일반도로, 2: 터널, 3: 교량, 4: 지하차도, 5: 고가차도
  std::vector<common::math::Vec3d> points_3d_;
  common::math::Polygon2d polygon_;
  std::vector<common::math::LineSegment2d> mean_point_based_segments_; // for kdtree construction
};

} // namespace hdmap
} // namespace keti