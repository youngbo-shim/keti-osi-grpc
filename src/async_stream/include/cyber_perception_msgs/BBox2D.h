// Generated by gencpp from file cyber_perception_msgs/BBox2D.msg
// DO NOT EDIT!


#ifndef CYBER_PERCEPTION_MSGS_MESSAGE_BBOX2D_H
#define CYBER_PERCEPTION_MSGS_MESSAGE_BBOX2D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cyber_perception_msgs
{
template <class ContainerAllocator>
struct BBox2D_
{
  typedef BBox2D_<ContainerAllocator> Type;

  BBox2D_()
    : xmin(0.0)
    , ymin(0.0)
    , xmax(0.0)
    , ymax(0.0)  {
    }
  BBox2D_(const ContainerAllocator& _alloc)
    : xmin(0.0)
    , ymin(0.0)
    , xmax(0.0)
    , ymax(0.0)  {
  (void)_alloc;
    }



   typedef double _xmin_type;
  _xmin_type xmin;

   typedef double _ymin_type;
  _ymin_type ymin;

   typedef double _xmax_type;
  _xmax_type xmax;

   typedef double _ymax_type;
  _ymax_type ymax;





  typedef boost::shared_ptr< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> const> ConstPtr;

}; // struct BBox2D_

typedef ::cyber_perception_msgs::BBox2D_<std::allocator<void> > BBox2D;

typedef boost::shared_ptr< ::cyber_perception_msgs::BBox2D > BBox2DPtr;
typedef boost::shared_ptr< ::cyber_perception_msgs::BBox2D const> BBox2DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cyber_perception_msgs::BBox2D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cyber_perception_msgs::BBox2D_<ContainerAllocator1> & lhs, const ::cyber_perception_msgs::BBox2D_<ContainerAllocator2> & rhs)
{
  return lhs.xmin == rhs.xmin &&
    lhs.ymin == rhs.ymin &&
    lhs.xmax == rhs.xmax &&
    lhs.ymax == rhs.ymax;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cyber_perception_msgs::BBox2D_<ContainerAllocator1> & lhs, const ::cyber_perception_msgs::BBox2D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cyber_perception_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2da0e8eb269cd8cc187fb4c7d113d543";
  }

  static const char* value(const ::cyber_perception_msgs::BBox2D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2da0e8eb269cd8ccULL;
  static const uint64_t static_value2 = 0x187fb4c7d113d543ULL;
};

template<class ContainerAllocator>
struct DataType< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cyber_perception_msgs/BBox2D";
  }

  static const char* value(const ::cyber_perception_msgs::BBox2D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 xmin  # in pixels.\n"
"float64 ymin  # in pixels.\n"
"float64 xmax  # in pixels.\n"
"float64 ymax  # in pixels.\n"
;
  }

  static const char* value(const ::cyber_perception_msgs::BBox2D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.xmin);
      stream.next(m.ymin);
      stream.next(m.xmax);
      stream.next(m.ymax);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BBox2D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cyber_perception_msgs::BBox2D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cyber_perception_msgs::BBox2D_<ContainerAllocator>& v)
  {
    s << indent << "xmin: ";
    Printer<double>::stream(s, indent + "  ", v.xmin);
    s << indent << "ymin: ";
    Printer<double>::stream(s, indent + "  ", v.ymin);
    s << indent << "xmax: ";
    Printer<double>::stream(s, indent + "  ", v.xmax);
    s << indent << "ymax: ";
    Printer<double>::stream(s, indent + "  ", v.ymax);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CYBER_PERCEPTION_MSGS_MESSAGE_BBOX2D_H
