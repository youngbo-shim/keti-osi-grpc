#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class HeightBarrierInfo
 *
 * @brief Information of height barrier in High-precision map.
 */
class HeightBarrierInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const std::string &link_id() const { return link_id_; }
  const unsigned int &type() const { return type_; }
  const unsigned int &ref_lane() const { return ref_lane_; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return segments_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_link_id(std::string link_id) { link_id_ = link_id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_ref_lane(unsigned int ref_lane) { ref_lane_ = ref_lane; }
  void set_points(std::vector<common::math::Vec2d> points) { points_ = points; }
  const double total_length() const { return total_length_; }
  void Init();

private:
  // Attributes
  std::string id_; // 고유 id
  std::string link_id_; // 높이 장애물과 주행경로 링크가 만나는 link id 
  unsigned int type_; // 시설 유형 - 1: 고가도로 또는 교량, 2: 육교, 4: 기타
  unsigned int ref_lane_; // 참조 차로 수
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