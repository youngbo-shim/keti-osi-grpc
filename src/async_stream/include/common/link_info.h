#pragma once

#include "common/common_info.h"
#include "common/lane_info.h"
#include "common/intersection_info.h"

namespace keti {
namespace hdmap {

/**
 * @class LinkInfo
 *
 * @brief Information of Links in High-precision map.
 */
class LinkInfo{
public:

  // Attributes
  const std::string &id() const { return id_; }  
  const std::string &from_node_id() const { return from_node_id_; }
  const std::string &to_node_id() const { return to_node_id_; }
  const std::string &left_link_id() const { return left_link_id_; }
  const std::string &right_link_id() const { return right_link_id_; }
  const std::string &ITS_link_id() const { return ITS_link_id_; }
  const std::vector<std::string> &next_link_ids() const { return next_link_ids_; }
  const std::vector<std::string> &prev_link_ids() const { return prev_link_ids_; }
  const double &length() const { return length_; }
  const unsigned char &speed_limit() const { return speed_limit_; }
  const unsigned char &lane_number() const { return lane_number_; }
  const unsigned char &link_type() const { return link_type_; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }
  const std::string &stop_id() const { return stop_id_; }
  const LaneTurn &turn_type() const { return turn_type_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_from_node_id(std::string from_node_id) { from_node_id_ = from_node_id; }
  void set_to_node_id(std::string to_node_id) { to_node_id_ = to_node_id; }
  void set_left_link_id(std::string left_link_id) { left_link_id_ = left_link_id; }
  void set_right_link_id(std::string right_link_id) { right_link_id_ = right_link_id; }
  void set_ITS_link_id(std::string ITS_link_id) { ITS_link_id_ = ITS_link_id; }
  void set_next_link_ids(std::vector<std::string> next_link_ids) { next_link_ids_ = next_link_ids; }
  void set_prev_link_ids(std::vector<std::string> prev_link_ids) { prev_link_ids_ = prev_link_ids; }
  void set_length(double length) { length_ = length; }
  void set_speed_limit(unsigned char speed_limit) { speed_limit_ = speed_limit; }
  void set_lane_number(unsigned char lane_number) { lane_number_ = lane_number; }
  void set_link_type(unsigned char link_type) { link_type_ = link_type; }
  void set_points(std::vector<common::math::Vec2d> points) { points_ = points; }
  void set_points_3d(std::vector<common::math::Vec3d> points_3d) { points_3d_ = points_3d; }
  void set_stop_id(std::string stop_id){stop_id_ = stop_id; }
  void set_turn_type(LaneTurn turn_type){ turn_type_ = turn_type; }

  // Access other link
  const NodeInfoPtr &from_node() const { return from_node_; }
  const NodeInfoPtr &to_node() const { return to_node_; }
  const LinkInfoPtr &left_link() const { return left_link_; }
  const LinkInfoPtr &right_link() const {return right_link_; }
  const std::vector<LinkInfoPtr> &next_links() const { return next_links_; }
  const std::vector<LinkInfoPtr> &prev_links() const { return prev_links_; }  
  const std::vector<LaneInfoPtr> &left_lanes() const { return left_lanes_; }
  const std::vector<LaneInfoPtr> &right_lanes() const { return right_lanes_; }  
  const StopInfoPtr &stop_lane() const { return stop_lane_; }  
  const InterSectionInfoPtr &overlapped_intersection_area() const { return overlapped_intersection_area_; }

  void set_from_node(NodeInfoPtr from_node) { from_node_ = from_node; }
  void set_to_node(NodeInfoPtr to_node) { to_node_ = to_node; }
  void set_left_link(LinkInfoPtr left_link) { left_link_ = left_link; }
  void set_right_link(LinkInfoPtr right_link) { right_link_ = right_link; }
  void set_next_links(std::vector<LinkInfoPtr> next_links) { next_links_ = next_links; }
  void set_prev_links(std::vector<LinkInfoPtr> prev_links) { prev_links_ = prev_links; }
  void set_next_link_id(std::string next_link_id){ next_link_ids_.push_back(next_link_id); }
  void set_prev_link_id(std::string prev_link_id){ prev_link_ids_.push_back(prev_link_id); }
  void set_left_lanes(LaneInfoPtr left_lane) { left_lanes_.push_back(left_lane); }
  void set_right_lanes(LaneInfoPtr right_lane) { right_lanes_.push_back(right_lane); }
  void set_stop_lane(StopInfoPtr stop_lane){ stop_lane_ = stop_lane; }
  void set_overlapped_intersection_area(InterSectionInfoPtr overlapped_intersection_area) { overlapped_intersection_area_ = overlapped_intersection_area; }
  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
  }

  const std::vector<double> &headings() const { return headings_; }
  const std::vector<common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  const std::vector<double> &accumulated_s() const { return accumulated_s_; }
  double total_length() const { return total_length_; }

  void Init();

  double GetLeftLane(const double s, LaneInfoPtr* left_lane) const;
  double GetRightLane(const double s, LaneInfoPtr* right_lane) const;

  void GetWidth(const double s, const InterSectionInfoPtr overlapped_intersection_area, 
                const common::math::LineSegment2d& point_projected_seg,
                double *left_width, double *right_width ) const;
  void GetWidth(const double s, double *left_width,
                double *right_width ) const;
  double GetWidth(const double s) const;
  common::math::Vec2d GetNearestPoint(const common::math::Vec2d &point,
                                      double *distance) const;
  double DistanceTo(const common::math::Vec2d &point,
                    LinkSegmentKDTree* segments) const;
  double DistanceTo(const common::math::Vec2d &point, common::math::Vec2d *map_point,
                    double *s_offset, int *s_offset_index,
                    LinkSegmentKDTree* segments) const;

  bool GetProjection(const common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;
  common::math::Vec2d GetSmoothPoint(double s) const;
  common::math::Vec3d GetSmooth3DPoint(double s) const;
  double Heading(const double s) const;

  bool IsOnLink(const common::math::Vec2d &point) const;
  bool IsOnLink(const common::math::Box2d &box) const;
  double Curvature(const double s) const;

  const OverlapsTable &speed_bumps() const {
    return speed_bumps_;
  }
  const OverlapsTable &surfacemarks() const {
    return surfacemarks_;
  }
  const OverlapsTable &crosswalk() const {
    return crosswalk_;
  }
  const OverlapsTable &traffic_light() const {
    return traffic_light_;
  }
  const OverlapsTable &intersection() const {
    return intersection_;
  }
  const OverlapsTable &u_turn_stop() const {
    return u_turn_stop_;
  }
  const OverlapsTable &creep_stop() const {
    return creep_stop_;
  }
 const OverlapsTable &drivewaysection() const {
    return drivewaysection_;
  }

  OverlapsTable *mutable_speed_bumps() { return &speed_bumps_; }
  OverlapsTable *mutable_surfacemarks() { return &surfacemarks_; }
  OverlapsTable *mutable_drivewaysection() { return &drivewaysection_; }
  OverlapsTable *mutable_crosswalk() { return &crosswalk_; }
  OverlapsTable *mutable_traffic_light() { return &traffic_light_; }
  OverlapsTable *mutable_intersection() { return &intersection_; }
  OverlapsTable *mutable_u_turn_stop() { return &u_turn_stop_;}
  OverlapsTable *mutable_creep_stop() { return &creep_stop_;}
private:
  // Attributes
  std::string id_;
  std::string from_node_id_;
  std::string to_node_id_;
  std::string left_link_id_;
  std::string right_link_id_;
  std::string ITS_link_id_;
  std::vector<std::string> next_link_ids_;
  std::vector<std::string> prev_link_ids_;
  double length_;
  unsigned char speed_limit_;
  unsigned char lane_number_;
  unsigned char link_type_;
  std::vector<common::math::Vec2d> points_;
  std::vector<common::math::Vec3d> points_3d_;
  std::string stop_id_;
  LaneTurn turn_type_;
  InterSectionInfoPtr overlapped_intersection_area_;

  // Access other link
  NodeInfoPtr from_node_;
  NodeInfoPtr to_node_;
  LinkInfoPtr left_link_;
  LinkInfoPtr right_link_;
  std::vector<LinkInfoPtr> next_links_;
  std::vector<LinkInfoPtr> prev_links_;
  std::vector<LaneInfoPtr> left_lanes_;
  std::vector<LaneInfoPtr> right_lanes_;
  StopInfoPtr stop_lane_;

  // Overlaps
  OverlapsTable speed_bumps_;
  OverlapsTable surfacemarks_;
  OverlapsTable crosswalk_;
  OverlapsTable traffic_light_;
  OverlapsTable intersection_;
  OverlapsTable u_turn_stop_;
  OverlapsTable creep_stop_;
  OverlapsTable drivewaysection_;

  // for KDTree	
  std::vector<common::math::LineSegment2d> segments_;

  std::vector<common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<double> accumulated_s_;
  double total_length_;
};

} // namespace hdmap
} // namespace keti
