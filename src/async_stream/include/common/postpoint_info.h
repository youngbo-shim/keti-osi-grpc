#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class PostPointInfo
 *
 * @brief Information of post point in High-precision map.
 */
class PostPointInfo{
public:

  // Attributes
  const std::string &id() const { return id_; }
  const unsigned int &type() const { return type_; }
  const common::math::Vec2d &point() const { return point_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_point(common::math::Vec2d point) { point_ = point; }
  
  void Init(){ segments_.emplace_back(point_, point_); }

private:
  // Attributes
  std::string id_; // 고유 id
  unsigned int type_; // 1: 신호기지주, 2: 교통표지지주, 9: 기타 
  common::math::Vec2d point_; 
  std::vector<common::math::LineSegment2d> segments_;
};

} // namespace hdmap
} // namespace keti