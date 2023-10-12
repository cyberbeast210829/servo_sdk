// Generated by gencpp from file motctrl_can/retrieve_indicatorRequest.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_RETRIEVE_INDICATORREQUEST_H
#define MOTCTRL_CAN_MESSAGE_RETRIEVE_INDICATORREQUEST_H


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
struct retrieve_indicatorRequest_
{
  typedef retrieve_indicatorRequest_<ContainerAllocator> Type;

  retrieve_indicatorRequest_()
    : indiID(0)  {
    }
  retrieve_indicatorRequest_(const ContainerAllocator& _alloc)
    : indiID(0)  {
  (void)_alloc;
    }



   typedef uint8_t _indiID_type;
  _indiID_type indiID;





  typedef boost::shared_ptr< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> const> ConstPtr;

}; // struct retrieve_indicatorRequest_

typedef ::motctrl_can::retrieve_indicatorRequest_<std::allocator<void> > retrieve_indicatorRequest;

typedef boost::shared_ptr< ::motctrl_can::retrieve_indicatorRequest > retrieve_indicatorRequestPtr;
typedef boost::shared_ptr< ::motctrl_can::retrieve_indicatorRequest const> retrieve_indicatorRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator2> & rhs)
{
  return lhs.indiID == rhs.indiID;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator1> & lhs, const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motctrl_can

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b0da97dc388132fcb9c73d78da5a3ba3";
  }

  static const char* value(const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb0da97dc388132fcULL;
  static const uint64_t static_value2 = 0xb9c73d78da5a3ba3ULL;
};

template<class ContainerAllocator>
struct DataType< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motctrl_can/retrieve_indicatorRequest";
  }

  static const char* value(const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 indiID\n"
;
  }

  static const char* value(const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.indiID);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct retrieve_indicatorRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motctrl_can::retrieve_indicatorRequest_<ContainerAllocator>& v)
  {
    s << indent << "indiID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.indiID);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_RETRIEVE_INDICATORREQUEST_H