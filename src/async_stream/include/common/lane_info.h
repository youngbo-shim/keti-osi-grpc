#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class LaneInfo
 *
 * @brief Information of Lanes in High-precision map.
 */
class LaneInfo{
public:

  // Attributes
  const std::string &id() const { return id_; }
  const unsigned int &lane_type() const { return lane_type_; }
  const unsigned int &kind() const { return kind_; }
  const unsigned int &barrier() const { return barrier_; }
  const std::string &hdufid() const { return hdufid_; }	
  const std::string &right_link_id() const { return right_link_id_; }
  const std::string &left_link_id() const { return left_link_id_; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }
  const std::vector<common::math::Vec3d> &points_3d() const { return points_3d_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_lane_type(unsigned int lane_type) { lane_type_ = lane_type; }
  void set_kind(unsigned int kind) { kind_ = kind; }
  void set_barrier(unsigned int barrier) { barrier_ = barrier; }
  void set_hdufid(std::string hdufid) { hdufid_= hdufid; }
  void set_right_link_id(std::string right_link_id) { right_link_id_= right_link_id; }
  void set_left_link_id(std::string left_link_id) { left_link_id_= left_link_id; }
  void set_points(std::vector<common::math::Vec2d> points) { points_ = points; }
  void set_points_3d(std::vector<common::math::Vec3d> points) { points_3d_ = points; }

  const std::vector<double> &headings() const { return headings_; }
  const std::vector<common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  const std::vector<double> &accumulated_s() const { return accumulated_s_; }
  double total_length() const { return total_length_; }
  
  void Init();

  bool GetProjection(const common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;

  common::math::Vec2d GetSmoothPoint(double s) const;
  common::math::Vec3d GetSmooth3DPoint(double s) const;
  double Heading(const double s) const;

  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
	}

  const bool CheckForbiddenLaneChangeType() const;

private:
  // Attributes
  std::string id_;
  unsigned int lane_type_;
  unsigned int kind_;
  unsigned int barrier_;
  std::string hdufid_;
  std::string right_link_id_;
  std::string left_link_id_;
  std::vector<common::math::Vec2d> points_;
  std::vector<common::math::Vec3d> points_3d_;

  // for KDTree
  std::vector<common::math::LineSegment2d> segments_;

  std::vector<common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<double> accumulated_s_;
  double total_length_;
};

} // namespace hdmap
} // namespace keti