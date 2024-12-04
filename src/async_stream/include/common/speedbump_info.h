#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class SpeedBumpInfo
 *
 * @brief Information of speed bump in High-precision map.
 */
class SpeedBumpInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const unsigned int &admin_code() const { return admin_code_; }
  const unsigned int &ref_lane() const { return ref_lane_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const common::math::Polygon2d &polygon() const { return polygon_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return mean_point_based_segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_admin_code(unsigned int admin_code) { admin_code_ = admin_code; }
  void set_ref_lane(unsigned int ref_lane) { ref_lane_ = ref_lane; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }
  void set_polygon_from_points(std::vector<common::math::Vec2d> points) { polygon_ = common::math::Polygon2d(points); }
  void InitSegmentWithMeanPoint(){ mean_point_based_segments_.emplace_back(polygon_.GetMeanPoint(), polygon_.GetMeanPoint()); }

private:
  // Attributes
  std::string id_;
  unsigned int admin_code_;
  unsigned int ref_lane_;
  std::vector<common::math::Vec3d> points_3d_;
  common::math::Polygon2d polygon_;
  std::vector<common::math::LineSegment2d> mean_point_based_segments_; // for kdtree construction
};

} // namespace hdmap
} // namespace keti