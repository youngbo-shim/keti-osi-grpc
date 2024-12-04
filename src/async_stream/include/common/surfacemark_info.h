#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class SurfaceMarkInfo
 *
 * @brief Information of surface mark in High-precision map.
 */
class SurfaceMarkInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const std::string &link_id() const { return link_id_; }
  const std::vector<std::string> &trafficlight_ids() const { return trafficlight_ids_; }
  const unsigned int &kind() const { return kind_; }
  const unsigned int &type() const { return type_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const common::math::Polygon2d &polygon() const { return polygon_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return mean_point_based_segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_link_id(std::string link_id) { link_id_ = link_id; }
  void set_trafficlight_ids(std::vector<std::string> trafficlight_ids){ trafficlight_ids_ = trafficlight_ids; }
  void set_kind(unsigned int kind) { kind_ = kind; }
  void set_type(unsigned int type) { type_ = type; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }
  void set_polygon_from_points(std::vector<common::math::Vec2d> points) { polygon_ = common::math::Polygon2d(points); }
  void InitSegmentWithMeanPoint(){ mean_point_based_segments_.emplace_back(polygon_.GetMeanPoint(), polygon_.GetMeanPoint()); }
private:
  // Attributes
  std::string id_; // 고유 id
  std::string link_id_; // 노면 표시와 연계되는 링크 id
  std::vector<std::string> trafficlight_ids_; // 노면 표시와 연계되는 신호등 id
  unsigned int kind_; // 표시 종류 - 5321: 횡단보도, 5391: 유턴....
  unsigned int type_;  // 표시 형태 - 1: 화살표, 5: 횡단보도
  std::vector<common::math::Vec3d> points_3d_;
  common::math::Polygon2d polygon_;
  std::vector<common::math::LineSegment2d> mean_point_based_segments_; // for kdtree construction
};

} // namespace hdmap
} // namespace keti