// Generated by gencpp from file control_msgs/VehicleState.msg
// DO NOT EDIT!


#ifndef CONTROL_MSGS_MESSAGE_VEHICLESTATE_H
#define CONTROL_MSGS_MESSAGE_VEHICLESTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace control_msgs
{
template <class ContainerAllocator>
struct VehicleState_
{
  typedef VehicleState_<ContainerAllocator> Type;

  VehicleState_()
    : header()
    , autonomous_status(0)
    , turn_signal(0)
    , gas_pedal_percentage(0)
    , brake_pedal_percentage(0)
    , gear_sel(0)
    , v_ego(0.0)
    , a_x(0.0)
    , yaw_rate(0.0)
    , steer_angle(0.0)
    , x_utm(0.0)
    , y_utm(0.0)
    , heading_utm(0.0)  {
    }
  VehicleState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , autonomous_status(0)
    , turn_signal(0)
    , gas_pedal_percentage(0)
    , brake_pedal_percentage(0)
    , gear_sel(0)
    , v_ego(0.0)
    , a_x(0.0)
    , yaw_rate(0.0)
    , steer_angle(0.0)
    , x_utm(0.0)
    , y_utm(0.0)
    , heading_utm(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int8_t _autonomous_status_type;
  _autonomous_status_type autonomous_status;

   typedef int8_t _turn_signal_type;
  _turn_signal_type turn_signal;

   typedef int8_t _gas_pedal_percentage_type;
  _gas_pedal_percentage_type gas_pedal_percentage;

   typedef int8_t _brake_pedal_percentage_type;
  _brake_pedal_percentage_type brake_pedal_percentage;

   typedef int8_t _gear_sel_type;
  _gear_sel_type gear_sel;

   typedef double _v_ego_type;
  _v_ego_type v_ego;

   typedef double _a_x_type;
  _a_x_type a_x;

   typedef double _yaw_rate_type;
  _yaw_rate_type yaw_rate;

   typedef double _steer_angle_type;
  _steer_angle_type steer_angle;

   typedef double _x_utm_type;
  _x_utm_type x_utm;

   typedef double _y_utm_type;
  _y_utm_type y_utm;

   typedef double _heading_utm_type;
  _heading_utm_type heading_utm;





  typedef boost::shared_ptr< ::control_msgs::VehicleState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_msgs::VehicleState_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleState_

typedef ::control_msgs::VehicleState_<std::allocator<void> > VehicleState;

typedef boost::shared_ptr< ::control_msgs::VehicleState > VehicleStatePtr;
typedef boost::shared_ptr< ::control_msgs::VehicleState const> VehicleStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_msgs::VehicleState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_msgs::VehicleState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_msgs::VehicleState_<ContainerAllocator1> & lhs, const ::control_msgs::VehicleState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.autonomous_status == rhs.autonomous_status &&
    lhs.turn_signal == rhs.turn_signal &&
    lhs.gas_pedal_percentage == rhs.gas_pedal_percentage &&
    lhs.brake_pedal_percentage == rhs.brake_pedal_percentage &&
    lhs.gear_sel == rhs.gear_sel &&
    lhs.v_ego == rhs.v_ego &&
    lhs.a_x == rhs.a_x &&
    lhs.yaw_rate == rhs.yaw_rate &&
    lhs.steer_angle == rhs.steer_angle &&
    lhs.x_utm == rhs.x_utm &&
    lhs.y_utm == rhs.y_utm &&
    lhs.heading_utm == rhs.heading_utm;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_msgs::VehicleState_<ContainerAllocator1> & lhs, const ::control_msgs::VehicleState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_msgs::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_msgs::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_msgs::VehicleState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_msgs::VehicleState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_msgs::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_msgs::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_msgs::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f13be268a076fffca43131339984a89b";
  }

  static const char* value(const ::control_msgs::VehicleState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf13be268a076fffcULL;
  static const uint64_t static_value2 = 0xa43131339984a89bULL;
};

template<class ContainerAllocator>
struct DataType< ::control_msgs::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_msgs/VehicleState";
  }

  static const char* value(const ::control_msgs::VehicleState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_msgs::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# Vehicle auto status & chassis info.\n"
"# Autonomous Status Info (0x00 ~ 0x0A) \n"
"# AUTONOMOUS_NONE, AUTONOMOUS_READY, AUTONOMOUS_ALL, AUTONOMOUS_ACC, AUTONOMOUS_STEER, AUTONOMOUS_CANCEL, AUTONOMOUS_CANCEL_EStop,\n"
"# AUTONOMOUS_CANCEL_ACC_PEDAL, AUTONOMOUS_CANCEL_BRAKE_PEDAL, AUTONOMOUS_CANCEL_STEER, AUTONOMOUS_ERROR\n"
"\n"
"int8 autonomous_status         # Autonomous Status.\n"
"int8 turn_signal               # Trun Signal(0x00: None, 0x01: right, 0x02: hazard, 0x04: left).\n"
"int8 gas_pedal_percentage      # Gas Pedal Pressing info. (0-145) \n"
"int8 brake_pedal_percentage    # Brake Pedal Pressing info. (0-130)\n"
"int8 gear_sel                  # Gear Selecting info. (0x00: P, 0x07: R, 0x06: N, 0x05: D)\n"
"\n"
"# Vehicle motion info.\n"
"float64 v_ego           # Current speed of vehicle to x-axis. (km/h)\n"
"float64 a_x             # Acceleration of vehicle to x-axis. (m/s^2)\n"
"float64 yaw_rate        # Yaw rate(angular velocity). (rad/s)\n"
"float64 steer_angle     # Steering Wheel Angle(degree).\n"
"\n"
"# Vehicle pose info.\n"
"float64 x_utm           # The x-coordinate of vehicle position. (m)\n"
"float64 y_utm           # The y-coordinate of vehicle position. (m)\n"
"float64 heading_utm     # The heading of vehicle position, which is the angle between the vehicle's heading direction and x-axis. (rad)\n"
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
;
  }

  static const char* value(const ::control_msgs::VehicleState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_msgs::VehicleState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.autonomous_status);
      stream.next(m.turn_signal);
      stream.next(m.gas_pedal_percentage);
      stream.next(m.brake_pedal_percentage);
      stream.next(m.gear_sel);
      stream.next(m.v_ego);
      stream.next(m.a_x);
      stream.next(m.yaw_rate);
      stream.next(m.steer_angle);
      stream.next(m.x_utm);
      stream.next(m.y_utm);
      stream.next(m.heading_utm);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_msgs::VehicleState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_msgs::VehicleState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "autonomous_status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.autonomous_status);
    s << indent << "turn_signal: ";
    Printer<int8_t>::stream(s, indent + "  ", v.turn_signal);
    s << indent << "gas_pedal_percentage: ";
    Printer<int8_t>::stream(s, indent + "  ", v.gas_pedal_percentage);
    s << indent << "brake_pedal_percentage: ";
    Printer<int8_t>::stream(s, indent + "  ", v.brake_pedal_percentage);
    s << indent << "gear_sel: ";
    Printer<int8_t>::stream(s, indent + "  ", v.gear_sel);
    s << indent << "v_ego: ";
    Printer<double>::stream(s, indent + "  ", v.v_ego);
    s << indent << "a_x: ";
    Printer<double>::stream(s, indent + "  ", v.a_x);
    s << indent << "yaw_rate: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_rate);
    s << indent << "steer_angle: ";
    Printer<double>::stream(s, indent + "  ", v.steer_angle);
    s << indent << "x_utm: ";
    Printer<double>::stream(s, indent + "  ", v.x_utm);
    s << indent << "y_utm: ";
    Printer<double>::stream(s, indent + "  ", v.y_utm);
    s << indent << "heading_utm: ";
    Printer<double>::stream(s, indent + "  ", v.heading_utm);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_MSGS_MESSAGE_VEHICLESTATE_H
