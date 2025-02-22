// Generated by gencpp from file control_msgs/VehicleCMD.msg
// DO NOT EDIT!


#ifndef CONTROL_MSGS_MESSAGE_VEHICLECMD_H
#define CONTROL_MSGS_MESSAGE_VEHICLECMD_H


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
struct VehicleCMD_
{
  typedef VehicleCMD_<ContainerAllocator> Type;

  VehicleCMD_()
    : header()
    , steer_angle_cmd(0.0)
    , accel_decel_cmd(0.0)
    , target_speed_cmd(0.0)
    , AEB_decel_cmd(0.0)  {
    }
  VehicleCMD_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , steer_angle_cmd(0.0)
    , accel_decel_cmd(0.0)
    , target_speed_cmd(0.0)
    , AEB_decel_cmd(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _steer_angle_cmd_type;
  _steer_angle_cmd_type steer_angle_cmd;

   typedef double _accel_decel_cmd_type;
  _accel_decel_cmd_type accel_decel_cmd;

   typedef double _target_speed_cmd_type;
  _target_speed_cmd_type target_speed_cmd;

   typedef double _AEB_decel_cmd_type;
  _AEB_decel_cmd_type AEB_decel_cmd;





  typedef boost::shared_ptr< ::control_msgs::VehicleCMD_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_msgs::VehicleCMD_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleCMD_

typedef ::control_msgs::VehicleCMD_<std::allocator<void> > VehicleCMD;

typedef boost::shared_ptr< ::control_msgs::VehicleCMD > VehicleCMDPtr;
typedef boost::shared_ptr< ::control_msgs::VehicleCMD const> VehicleCMDConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_msgs::VehicleCMD_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_msgs::VehicleCMD_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_msgs::VehicleCMD_<ContainerAllocator1> & lhs, const ::control_msgs::VehicleCMD_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.steer_angle_cmd == rhs.steer_angle_cmd &&
    lhs.accel_decel_cmd == rhs.accel_decel_cmd &&
    lhs.target_speed_cmd == rhs.target_speed_cmd &&
    lhs.AEB_decel_cmd == rhs.AEB_decel_cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_msgs::VehicleCMD_<ContainerAllocator1> & lhs, const ::control_msgs::VehicleCMD_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_msgs::VehicleCMD_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_msgs::VehicleCMD_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_msgs::VehicleCMD_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_msgs::VehicleCMD_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_msgs::VehicleCMD_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_msgs::VehicleCMD_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_msgs::VehicleCMD_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cabfc37a60e4dd2db12de73db2f19940";
  }

  static const char* value(const ::control_msgs::VehicleCMD_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcabfc37a60e4dd2dULL;
  static const uint64_t static_value2 = 0xb12de73db2f19940ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_msgs::VehicleCMD_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_msgs/VehicleCMD";
  }

  static const char* value(const ::control_msgs::VehicleCMD_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_msgs::VehicleCMD_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float64 steer_angle_cmd		#0-5000 (This value is scaled at 10)\n"
"float64 accel_decel_cmd		#0-2047 (bias : 1023, scale : 0.01)\n"
"float64 target_speed_cmd	#0-120 	(bias : 120, scale : 1)\n"
"float64 AEB_decel_cmd		#0-100  (bias : 0, scale : 0.01)\n"
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

  static const char* value(const ::control_msgs::VehicleCMD_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_msgs::VehicleCMD_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.steer_angle_cmd);
      stream.next(m.accel_decel_cmd);
      stream.next(m.target_speed_cmd);
      stream.next(m.AEB_decel_cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleCMD_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_msgs::VehicleCMD_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_msgs::VehicleCMD_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "steer_angle_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.steer_angle_cmd);
    s << indent << "accel_decel_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.accel_decel_cmd);
    s << indent << "target_speed_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.target_speed_cmd);
    s << indent << "AEB_decel_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.AEB_decel_cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_MSGS_MESSAGE_VEHICLECMD_H
