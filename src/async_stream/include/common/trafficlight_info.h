#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class TrafficLightInfo
 *
 * @brief Information of traffic light in High-precision map.
 */
class TrafficLightInfo{
public:

  TrafficLightInfo() { intersection_id_= "UNKNOWN";}
  // Attributes
  const std::string &id() const { return id_; }
  const std::string &link_id() const { return link_id_; }
  const unsigned int &type() const { return type_; }
  const unsigned int &ref_lane() const { return ref_lane_; }
  const std::string &post_id() const { return post_id_; }
  const common::math::Vec3d &point() const { return point_; }
  const std::vector<common::math::Vec3d> &points() const { return points_; }
  const std::vector<common::math::LineSegment2d> &segments() const { return segments_; }
  const std::string intersection_id() const { return intersection_id_; }
  const double &heading() const { return heading_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_link_id(std::string link_id) { link_id_ = link_id; }
  void set_type(unsigned int type) { type_ = type; }
  void set_ref_lane(unsigned int ref_lane) { ref_lane_ = ref_lane; }
  void set_post_id(std::string post_id) { post_id_ = post_id; }
  void set_point(common::math::Vec3d point) { point_ = point; }
  void set_points(std::vector<common::math::Vec3d> points) { points_ = points; }
  void set_intersection_id(std::string intersection_id) { intersection_id_ = intersection_id; }
  void set_heading(double heading) { heading_ = heading; }

  void Init(){
    segments_.clear();
    if (points_.size() > 1){
      for (size_t i = 0; i + 1 < points_.size(); ++i){
        segments_.emplace_back(common::math::Vec2d(points_[i].x(), points_[i].y()),
                               common::math::Vec2d(points_[i+1].x(), points_[i+1].y()));
      }
    }
    else{
      segments_.emplace_back(common::math::Vec2d(point_.x(), point_.y()),
                             common::math::Vec2d(point_.x(), point_.y()));
    }
  }

private:
  // Attributes
  std::string id_; // 고유 id
  std::string link_id_; // 신호등과 연계되는 링크 id
  unsigned int type_; // 1: 차량횡형-삼색등, 2: 차량횡형-사색등 ... 99: 기타 신호등 유형
  unsigned int ref_lane_; // 참조해야하는 차로 수
  std::string post_id_; // 해당지주 id
  std::string intersection_id_; // traffic light과 인접한 교차로 id
  double heading_;
  common::math::Vec3d point_;
  std::vector<common::math::Vec3d> points_;

  std::vector<common::math::LineSegment2d> segments_;
};

} // namespace hdmap
} // namespace keti