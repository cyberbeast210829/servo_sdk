// Generated by gencpp from file motctrl_can/torque_controlRequest.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_TORQUE_CONTROLREQUEST_H
#define MOTCTRL_CAN_MESSAGE_TORQUE_CONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motctrl_can
{
template <class ContainerAllocator>
struct torque_controlRequest_
{
  typedef torque_controlRequest_<ContainerAllocator> Type;

  torque_controlRequest_()
    : torque(0.0)
    , duration(0)  {
    }
  torque_controlRequest_(const ContainerAllocator& _alloc)
    : torque(0.0)
    , duration(0)  {
  (void)_alloc;
    }



   typedef float _torque_type;
  _torque_type torque;

   typedef uint32_t _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::motctrl_can::torque_controlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motctrl_can::torque_controlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct torque_controlRequest_

typedef ::motctrl_can::torque_controlRequest_<std::allocator<void> > torque_controlRequest;

typedef boost::shared_ptr< ::motctrl_can::torque_controlRequest > torque_controlRequestPtr;
typedef boost::shared_ptr< ::motctrl_can::torque_controlRequest const> torque_controlRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motctrl_can::torque_controlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motctrl_can::torque_controlRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::torque_controlRequest_<ContainerAllocator2> & rhs)
{
  return lhs.torque == rhs.torque &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motctrl_can::torque_controlRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::torque_controlRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motctrl_can

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::torque_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::torque_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::torque_controlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "def1487aa00729fab0b3d8efa078a330";
  }

  static const char* value(const ::motctrl_can::torque_controlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdef1487aa00729faULL;
  static const uint64_t static_value2 = 0xb0b3d8efa078a330ULL;
};

template<class ContainerAllocator>
struct DataType< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motctrl_can/torque_controlRequest";
  }

  static const char* value(const ::motctrl_can::torque_controlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 torque\n"
"uint32 duration\n"
;
  }

  static const char* value(const ::motctrl_can::torque_controlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.torque);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct torque_controlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motctrl_can::torque_controlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motctrl_can::torque_controlRequest_<ContainerAllocator>& v)
  {
    s << indent << "torque: ";
    Printer<float>::stream(s, indent + "  ", v.torque);
    s << indent << "duration: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_TORQUE_CONTROLREQUEST_H
