#pragma once

#include "common/common_info.h"

namespace keti {
namespace hdmap {

/**
 * @class NodeInfo
 *
 * @brief Information of Nodes in High-precision map.
 */
class NodeInfo{
public:

  // Attributes
  const std::string &id() const { return id_; }
  const unsigned char &node_type() const { return node_type_; }
  const std::vector<std::string> &prev_link_ids() const { return prev_link_ids_; }
  const std::vector<std::string> &next_link_ids() const { return next_link_ids_; }	
  const common::math::Vec2d &point() const { return point_; }

  // Set Attributes
  void set_id(std::string id) { id_ = id; }
  void set_node_type(unsigned char node_type) { node_type_ = node_type; }
  void set_prev_link_ids(std::vector<std::string> prev_link_ids) { prev_link_ids_ = prev_link_ids; }
  void set_next_link_ids(std::vector<std::string> next_link_ids) { next_link_ids_ = next_link_ids; }
  void set_point(common::math::Vec2d point) { point_ = point; }

  // Access other link
  const std::vector<NodeInfoPtr> &prev_nodes() const { return prev_nodes_; }
  const std::vector<NodeInfoPtr> &next_nodes() const { return next_nodes_; }
  const std::vector<LinkInfoPtr> &prev_links() const { return prev_links_; }
  const std::vector<LinkInfoPtr> &next_links() const { return next_links_; }

  void set_prev_nodes(std::vector<NodeInfoPtr> prev_nodes) { prev_nodes_ = prev_nodes; }
  void set_next_nodes(std::vector<NodeInfoPtr> next_nodes) { next_nodes_ = next_nodes; }
  void set_prev_links(std::vector<LinkInfoPtr> prev_links) { prev_links_ = prev_links; }
  void set_next_links(std::vector<LinkInfoPtr> next_links) { next_links_ = next_links; }
  void set_next_link_id(std::string next_link_id){ next_link_ids_.push_back(next_link_id); }
  void set_prev_link_id(std::string prev_link_id){ prev_link_ids_.push_back(prev_link_id); }

  void Init(){		
    segments_.emplace_back(point_, point_);
  };

  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
  }

private:
  // Attributes
  std::string id_;	
  unsigned char node_type_;
  std::vector<std::string> prev_link_ids_;
  std::vector<std::string> next_link_ids_;
  common::math::Vec2d point_;

  // Access other link
  std::vector<NodeInfoPtr> prev_nodes_;
  std::vector<NodeInfoPtr> next_nodes_;	
  std::vector<LinkInfoPtr> prev_links_;
  std::vector<LinkInfoPtr> next_links_;

  std::vector<common::math::LineSegment2d> segments_;
};

} // namespace hdmap
} // namespace keti