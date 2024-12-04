#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class StopInfo
 *
 * @brief Information of Stop Line in High-precision map.
 */
class StopInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_points(std::vector<common::math::Vec2d> points) { points_ = points; }


  void Init(){
    for ( size_t i = 0; i + 1 < points_.size(); ++i )
      segments_.emplace_back(points_[i], points_[i + 1]);
  };

  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
  }

private:
  // Attributes
  std::string id_;
  std::vector<common::math::Vec2d> points_;

  // for KDTree
  std::vector<common::math::LineSegment2d> segments_;
};

} // namespace hdmap
} // namespace keti