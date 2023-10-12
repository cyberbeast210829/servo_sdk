// Generated by gencpp from file motctrl_can/modify_configurationRequest.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_MODIFY_CONFIGURATIONREQUEST_H
#define MOTCTRL_CAN_MESSAGE_MODIFY_CONFIGURATIONREQUEST_H


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
struct modify_configurationRequest_
{
  typedef modify_configurationRequest_<ContainerAllocator> Type;

  modify_configurationRequest_()
    : confType(0)
    , confID(0)
    , confData(0.0)  {
    }
  modify_configurationRequest_(const ContainerAllocator& _alloc)
    : confType(0)
    , confID(0)
    , confData(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _confType_type;
  _confType_type confType;

   typedef uint8_t _confID_type;
  _confID_type confID;

   typedef float _confData_type;
  _confData_type confData;





  typedef boost::shared_ptr< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct modify_configurationRequest_

typedef ::motctrl_can::modify_configurationRequest_<std::allocator<void> > modify_configurationRequest;

typedef boost::shared_ptr< ::motctrl_can::modify_configurationRequest > modify_configurationRequestPtr;
typedef boost::shared_ptr< ::motctrl_can::modify_configurationRequest const> modify_configurationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motctrl_can::modify_configurationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motctrl_can::modify_configurationRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::modify_configurationRequest_<ContainerAllocator2> & rhs)
{
  return lhs.confType == rhs.confType &&
    lhs.confID == rhs.confID &&
    lhs.confData == rhs.confData;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motctrl_can::modify_configurationRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::modify_configurationRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motctrl_can

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b3c09261b85a9983be1a2a09bec20e30";
  }

  static const char* value(const ::motctrl_can::modify_configurationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb3c09261b85a9983ULL;
  static const uint64_t static_value2 = 0xbe1a2a09bec20e30ULL;
};

template<class ContainerAllocator>
struct DataType< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motctrl_can/modify_configurationRequest";
  }

  static const char* value(const ::motctrl_can::modify_configurationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 confType\n"
"uint8 confID\n"
"float32 confData\n"
;
  }

  static const char* value(const ::motctrl_can::modify_configurationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.confType);
      stream.next(m.confID);
      stream.next(m.confData);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct modify_configurationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motctrl_can::modify_configurationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motctrl_can::modify_configurationRequest_<ContainerAllocator>& v)
  {
    s << indent << "confType: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confType);
    s << indent << "confID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confID);
    s << indent << "confData: ";
    Printer<float>::stream(s, indent + "  ", v.confData);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_MODIFY_CONFIGURATIONREQUEST_H