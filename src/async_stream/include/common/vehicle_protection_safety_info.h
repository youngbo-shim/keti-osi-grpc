#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class VehicleProtectionSafetyInfo
 *
 * @brief Information of vehicle protection safety in High-precision map.
 */
class VehicleProtectionSafetyInfo{
public:

  // Attributes
  const std::string &id() const { return id_; }
  const std::string &ref_id() const { return ref_id_; }
  const unsigned int &type() const { return type_; }
  const bool &is_central() const {return is_central_;}
  const unsigned int &low_high() const { return low_high_; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_ref_id(std::string ref_id) { ref_id_ = ref_id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_is_central(bool is_central) { is_central_ = is_central; }
  void set_low_high(unsigned int low_high) { low_high_ = low_high; }
  void set_points(std::vector<common::math::Vec2d> points) { points_ = points; }

  const double total_length() const { return total_length_; }
  void Init();
  common::math::Vec2d GetSmoothPoint(double s) const;

private:
  // Attributes
  std::string id_; // 고유 id
  std::string ref_id_; // 분리대 상 or 하단 id 
  unsigned int type_; // 시설 유형 - 2: 가드레일, 3: 콘크리트 방호벽 .. 99: 기타 시설물
  bool is_central_; // 중앙분리대 여부
  unsigned int low_high_; // 1: 상단, 2: 하단
  std::vector<common::math::Vec2d> points_; 

  // for KDTree
  std::vector<common::math::LineSegment2d> segments_;

  std::vector<common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<double> accumulated_s_;
  double total_length_;
};

} // namespace hdmap
} // namespace keti