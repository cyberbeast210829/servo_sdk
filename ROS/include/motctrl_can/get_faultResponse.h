// Generated by gencpp from file motctrl_can/get_faultResponse.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_GET_FAULTRESPONSE_H
#define MOTCTRL_CAN_MESSAGE_GET_FAULTRESPONSE_H


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
struct get_faultResponse_
{
  typedef get_faultResponse_<ContainerAllocator> Type;

  get_faultResponse_()
    : res(0)
    , faultNo(0)  {
    }
  get_faultResponse_(const ContainerAllocator& _alloc)
    : res(0)
    , faultNo(0)  {
  (void)_alloc;
    }



   typedef uint8_t _res_type;
  _res_type res;

   typedef uint8_t _faultNo_type;
  _faultNo_type faultNo;





  typedef boost::shared_ptr< ::motctrl_can::get_faultResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motctrl_can::get_faultResponse_<ContainerAllocator> const> ConstPtr;

}; // struct get_faultResponse_

typedef ::motctrl_can::get_faultResponse_<std::allocator<void> > get_faultResponse;

typedef boost::shared_ptr< ::motctrl_can::get_faultResponse > get_faultResponsePtr;
typedef boost::shared_ptr< ::motctrl_can::get_faultResponse const> get_faultResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motctrl_can::get_faultResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motctrl_can::get_faultResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motctrl_can::get_faultResponse_<ContainerAllocator1> & lhs, const ::motctrl_can::get_faultResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res &&
    lhs.faultNo == rhs.faultNo;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motctrl_can::get_faultResponse_<ContainerAllocator1> & lhs, const ::motctrl_can::get_faultResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motctrl_can

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motctrl_can::get_faultResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motctrl_can::get_faultResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motctrl_can::get_faultResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1788c58a36cb063b2c61986779b5b3b";
  }

  static const char* value(const ::motctrl_can::get_faultResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1788c58a36cb063ULL;
  static const uint64_t static_value2 = 0xb2c61986779b5b3bULL;
};

template<class ContainerAllocator>
struct DataType< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motctrl_can/get_faultResponse";
  }

  static const char* value(const ::motctrl_can::get_faultResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 res\n"
"uint8 faultNo\n"
;
  }

  static const char* value(const ::motctrl_can::get_faultResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
      stream.next(m.faultNo);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct get_faultResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motctrl_can::get_faultResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motctrl_can::get_faultResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.res);
    s << indent << "faultNo: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.faultNo);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_GET_FAULTRESPONSE_H