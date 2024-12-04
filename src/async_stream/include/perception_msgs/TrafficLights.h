// Generated by gencpp from file perception_msgs/TrafficLights.msg
// DO NOT EDIT!


#ifndef PERCEPTION_MSGS_MESSAGE_TRAFFICLIGHTS_H
#define PERCEPTION_MSGS_MESSAGE_TRAFFICLIGHTS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <perception_msgs/TrafficLight.h>

namespace perception_msgs
{
template <class ContainerAllocator>
struct TrafficLights_
{
  typedef TrafficLights_<ContainerAllocator> Type;

  TrafficLights_()
    : header()
    , traffic_lights()  {
    }
  TrafficLights_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , traffic_lights(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::perception_msgs::TrafficLight_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::perception_msgs::TrafficLight_<ContainerAllocator> >> _traffic_lights_type;
  _traffic_lights_type traffic_lights;





  typedef boost::shared_ptr< ::perception_msgs::TrafficLights_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::perception_msgs::TrafficLights_<ContainerAllocator> const> ConstPtr;

}; // struct TrafficLights_

typedef ::perception_msgs::TrafficLights_<std::allocator<void> > TrafficLights;

typedef boost::shared_ptr< ::perception_msgs::TrafficLights > TrafficLightsPtr;
typedef boost::shared_ptr< ::perception_msgs::TrafficLights const> TrafficLightsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::perception_msgs::TrafficLights_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::perception_msgs::TrafficLights_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::perception_msgs::TrafficLights_<ContainerAllocator1> & lhs, const ::perception_msgs::TrafficLights_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.traffic_lights == rhs.traffic_lights;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::perception_msgs::TrafficLights_<ContainerAllocator1> & lhs, const ::perception_msgs::TrafficLights_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace perception_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::perception_msgs::TrafficLights_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::perception_msgs::TrafficLights_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::perception_msgs::TrafficLights_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::perception_msgs::TrafficLights_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_msgs::TrafficLights_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_msgs::TrafficLights_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::perception_msgs::TrafficLights_<ContainerAllocator> >
{
  static const char* value()
  {
    return "494618a9b0c1837b324806a125567f81";
  }

  static const char* value(const ::perception_msgs::TrafficLights_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x494618a9b0c1837bULL;
  static const uint64_t static_value2 = 0x324806a125567f81ULL;
};

template<class ContainerAllocator>
struct DataType< ::perception_msgs::TrafficLights_<ContainerAllocator> >
{
  static const char* value()
  {
    return "perception_msgs/TrafficLights";
  }

  static const char* value(const ::perception_msgs::TrafficLights_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::perception_msgs::TrafficLights_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"perception_msgs/TrafficLight[] traffic_lights\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: perception_msgs/TrafficLight\n"
"uint8 HORIZONTAL_TRICOLOR = 1               # 차량횡형-삼색등\n"
"uint8 HORIZONTAL_FOURCOLORA = 2             # 차량횡형-사색등A\n"
"uint8 HORIZONTAL_FOURCOLORB = 3             # 차량횡형-사색등B\n"
"uint8 HORIZONTAL_ARROW_TRICOLOR = 4         # 차량횡형-화살표삼색등\n"
"uint8 VERTICAL_TRICOLOR = 5                 # 차량종형-삼색등\n"
"uint8 ERTICAL_ARROW_TRICOLOR = 6            # 차량종형-화살표삼색등\n"
"uint8 VERTICAL_FOURCOLOR = 7                # 차량종형-사색등\n"
"uint8 BUS_TRICOLOR = 8                      # 버스삼색등\n"
"uint8 VARIABLE_TRAFFIC_LANE_CONTROL = 9     # 가변형 가변등\n"
"uint8 VARIABLE_ALARM = 10                   # 경보형 가변등\n"
"uint8 PEDESTRIAN = 11                       # 보행등\n"
"uint8 VERTICAL_BICYCLE_TRICOLOR = 12        # 자전거종형-삼색등\n"
"uint8 VERTICAL_BICYCLE_TWOCOLOR = 13        # 자전거종형-이색등\n"
"uint8 VERTICCAL_AUXILIARY_TRICOLOR = 14     # 차량보조등-종형삼색등\n"
"uint8 VERTICCAL_AUXILIARY_FOURCOLOR = 15    # 차량보조등-종형사색등\n"
"uint8 OTHERS_TRAFFIC_LIGHT_TYPE = 99        # 기타 신호등 유형\n"
"\n"
"string id\n"
"string signal_group_id\n"
"uint8 type\n"
"perception_msgs/TrafficSignalPhase signal_phase\n"
"perception_msgs/TrafficSignalPhase next_signal_phase\n"
"float64 remaining_time\n"
"geometry_msgs/Point point\n"
"float64 heading\n"
"================================================================================\n"
"MSG: perception_msgs/TrafficSignalPhase\n"
"uint8 OFF = 0\n"
"uint8 UNKNOWN = 1\n"
"uint8 GREEN = 2\n"
"uint8 GREEN_LEFT = 3\n"
"uint8 RED = 4\n"
"uint8 RED_LEFT = 5\n"
"uint8 YELLOW = 6\n"
"uint8 RED_YELLOW = 7\n"
"uint8 YELLOW21 = 8\n"
"uint8 YELLOW_GREEN4 = 9\n"
"uint8 YELLOW_OTHER = 10\n"
"uint8 PEDESTRIAN = 11\n"
"uint8 NONE = 99\n"
"\n"
"uint8 signal_phase\n"
"bool blink\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::perception_msgs::TrafficLights_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::perception_msgs::TrafficLights_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.traffic_lights);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrafficLights_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::perception_msgs::TrafficLights_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::perception_msgs::TrafficLights_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "traffic_lights[]" << std::endl;
    for (size_t i = 0; i < v.traffic_lights.size(); ++i)
    {
      s << indent << "  traffic_lights[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::perception_msgs::TrafficLight_<ContainerAllocator> >::stream(s, indent + "    ", v.traffic_lights[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PERCEPTION_MSGS_MESSAGE_TRAFFICLIGHTS_H
