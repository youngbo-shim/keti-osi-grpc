#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class SafetySignInfo
 *
 * @brief Information of safety sign in High-precision map.
 */
class SafetySignInfo{
public:
  // Attributes
  const std::string &id() const { return id_; }
  const std::string &link_id() const { return link_id_; }
  const unsigned int &type() const { return type_; }
  const unsigned int &sub_type() const { return sub_type_; }
  const unsigned int &ref_lane() const { return ref_lane_; }
  const std::string &post_id() const { return post_id_; }
  const common::math::Vec2d &point() const { return point_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return segments_; }
  const double &heading() const { return heading_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_link_id(std::string link_id) { link_id_ = link_id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_sub_type(unsigned int sub_type) { sub_type_ = sub_type; }
  void set_ref_lane(unsigned int ref_lane) { ref_lane_ = ref_lane; }
  void set_post_id(std::string post_id) { post_id_ = post_id; }
  void set_point(common::math::Vec2d point) { point_ = point; }
  void set_points(std::vector<common::math::Vec3d> points_3d ) { points_3d_ = points_3d ; }
  void set_heading(double heading) { heading_ = heading; }

  void Init(){
    segments_.clear();
    if (points_3d_.size() > 1){
      for (size_t i = 0; i + 1 < points_3d_.size(); ++i){
       segments_.emplace_back(common::math::Vec2d(points_3d_[i].x(), points_3d_[i].y()),
                              common::math::Vec2d(points_3d_[i+1].x(), points_3d_[i+1].y()));}
    }
    else{
      segments_.emplace_back(common::math::Vec2d(point_.x(), point_.y()),
                             common::math::Vec2d(point_.x(), point_.y()));
    }
  };

private:
  // Attributes
  std::string id_; // 고유 id
  std::string link_id_; // 안전표지판과 연계되는 링크 id
  unsigned int type_; // 1:주의, 2:규제, 3:지시, 4:보조 표지
  unsigned int sub_type_;  // 199: 기타 
  unsigned int ref_lane_; // 참조해야하는 차로 수
  std::string post_id_; // 해당지주 id

  double heading_;
  common::math::Vec2d point_;
  std::vector<common::math::Vec3d> points_3d_;
  std::vector<common::math::LineSegment2d> segments_;
};

} // namespace hdmap
} // namespace keti