// Generated by gencpp from file cyber_perception_msgs/LaneMarkers.msg
// DO NOT EDIT!


#ifndef CYBER_PERCEPTION_MSGS_MESSAGE_LANEMARKERS_H
#define CYBER_PERCEPTION_MSGS_MESSAGE_LANEMARKERS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <cyber_perception_msgs/LaneMarker.h>
#include <cyber_perception_msgs/LaneMarker.h>
#include <cyber_perception_msgs/LaneMarker.h>
#include <cyber_perception_msgs/LaneMarker.h>

namespace cyber_perception_msgs
{
template <class ContainerAllocator>
struct LaneMarkers_
{
  typedef LaneMarkers_<ContainerAllocator> Type;

  LaneMarkers_()
    : left_lane_marker()
    , right_lane_marker()
    , next_left_lane_marker()
    , next_right_lane_marker()  {
    }
  LaneMarkers_(const ContainerAllocator& _alloc)
    : left_lane_marker(_alloc)
    , right_lane_marker(_alloc)
    , next_left_lane_marker(_alloc)
    , next_right_lane_marker(_alloc)  {
  (void)_alloc;
    }



   typedef  ::cyber_perception_msgs::LaneMarker_<ContainerAllocator>  _left_lane_marker_type;
  _left_lane_marker_type left_lane_marker;

   typedef  ::cyber_perception_msgs::LaneMarker_<ContainerAllocator>  _right_lane_marker_type;
  _right_lane_marker_type right_lane_marker;

   typedef std::vector< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >> _next_left_lane_marker_type;
  _next_left_lane_marker_type next_left_lane_marker;

   typedef std::vector< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >> _next_right_lane_marker_type;
  _next_right_lane_marker_type next_right_lane_marker;





  typedef boost::shared_ptr< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> const> ConstPtr;

}; // struct LaneMarkers_

typedef ::cyber_perception_msgs::LaneMarkers_<std::allocator<void> > LaneMarkers;

typedef boost::shared_ptr< ::cyber_perception_msgs::LaneMarkers > LaneMarkersPtr;
typedef boost::shared_ptr< ::cyber_perception_msgs::LaneMarkers const> LaneMarkersConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator1> & lhs, const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator2> & rhs)
{
  return lhs.left_lane_marker == rhs.left_lane_marker &&
    lhs.right_lane_marker == rhs.right_lane_marker &&
    lhs.next_left_lane_marker == rhs.next_left_lane_marker &&
    lhs.next_right_lane_marker == rhs.next_right_lane_marker;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator1> & lhs, const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cyber_perception_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9a805aada0f0c69f2e3d7ccb30e1f603";
  }

  static const char* value(const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9a805aada0f0c69fULL;
  static const uint64_t static_value2 = 0x2e3d7ccb30e1f603ULL;
};

template<class ContainerAllocator>
struct DataType< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cyber_perception_msgs/LaneMarkers";
  }

  static const char* value(const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "LaneMarker left_lane_marker\n"
"LaneMarker right_lane_marker\n"
"LaneMarker[] next_left_lane_marker\n"
"LaneMarker[] next_right_lane_marker\n"
"================================================================================\n"
"MSG: cyber_perception_msgs/LaneMarker\n"
"LaneBoundaryType lane_type\n"
"\n"
"float64 quality  # range = [0,1]; 1 = the best quality\n"
"int32 model_degree\n"
"\n"
"# equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0\n"
"float64 c0_position\n"
"float64 c1_heading_angle\n"
"float64 c2_curvature\n"
"float64 c3_curvature_derivative\n"
"float64 view_range\n"
"float64 longitude_start\n"
"float64 longitude_end\n"
"================================================================================\n"
"MSG: cyber_perception_msgs/LaneBoundaryType\n"
"uint8  UNKNOWN = 0\n"
"uint8  DOTTED_YELLOW = 1\n"
"uint8  DOTTED_WHITE = 2\n"
"uint8  SOLID_YELLOW = 3\n"
"uint8  SOLID_WHITE = 4\n"
"uint8  DOUBLE_YELLOW = 5\n"
"uint8  CURB = 6\n"
"\n"
"# Offset relative to the starting point of boundary\n"
"float64 s\n"
"# support multiple types\n"
"uint8[] types\n"
;
  }

  static const char* value(const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left_lane_marker);
      stream.next(m.right_lane_marker);
      stream.next(m.next_left_lane_marker);
      stream.next(m.next_right_lane_marker);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LaneMarkers_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cyber_perception_msgs::LaneMarkers_<ContainerAllocator>& v)
  {
    s << indent << "left_lane_marker: ";
    s << std::endl;
    Printer< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >::stream(s, indent + "  ", v.left_lane_marker);
    s << indent << "right_lane_marker: ";
    s << std::endl;
    Printer< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >::stream(s, indent + "  ", v.right_lane_marker);
    s << indent << "next_left_lane_marker[]" << std::endl;
    for (size_t i = 0; i < v.next_left_lane_marker.size(); ++i)
    {
      s << indent << "  next_left_lane_marker[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >::stream(s, indent + "    ", v.next_left_lane_marker[i]);
    }
    s << indent << "next_right_lane_marker[]" << std::endl;
    for (size_t i = 0; i < v.next_right_lane_marker.size(); ++i)
    {
      s << indent << "  next_right_lane_marker[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::cyber_perception_msgs::LaneMarker_<ContainerAllocator> >::stream(s, indent + "    ", v.next_right_lane_marker[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CYBER_PERCEPTION_MSGS_MESSAGE_LANEMARKERS_H
