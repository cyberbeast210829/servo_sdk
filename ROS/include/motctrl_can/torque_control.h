// Generated by gencpp from file motctrl_can/torque_control.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_TORQUE_CONTROL_H
#define MOTCTRL_CAN_MESSAGE_TORQUE_CONTROL_H

#include <ros/service_traits.h>


#include <motctrl_can/torque_controlRequest.h>
#include <motctrl_can/torque_controlResponse.h>


namespace motctrl_can
{

struct torque_control
{

typedef torque_controlRequest Request;
typedef torque_controlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct torque_control
} // namespace motctrl_can


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motctrl_can::torque_control > {
  static const char* value()
  {
    return "f1470ebeb43f5c9d9f23f7ca75d048c9";
  }

  static const char* value(const ::motctrl_can::torque_control&) { return value(); }
};

template<>
struct DataType< ::motctrl_can::torque_control > {
  static const char* value()
  {
    return "motctrl_can/torque_control";
  }

  static const char* value(const ::motctrl_can::torque_control&) { return value(); }
};


// service_traits::MD5Sum< ::motctrl_can::torque_controlRequest> should match
// service_traits::MD5Sum< ::motctrl_can::torque_control >
template<>
struct MD5Sum< ::motctrl_can::torque_controlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::torque_control >::value();
  }
  static const char* value(const ::motctrl_can::torque_controlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::torque_controlRequest> should match
// service_traits::DataType< ::motctrl_can::torque_control >
template<>
struct DataType< ::motctrl_can::torque_controlRequest>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::torque_control >::value();
  }
  static const char* value(const ::motctrl_can::torque_controlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motctrl_can::torque_controlResponse> should match
// service_traits::MD5Sum< ::motctrl_can::torque_control >
template<>
struct MD5Sum< ::motctrl_can::torque_controlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::torque_control >::value();
  }
  static const char* value(const ::motctrl_can::torque_controlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::torque_controlResponse> should match
// service_traits::DataType< ::motctrl_can::torque_control >
template<>
struct DataType< ::motctrl_can::torque_controlResponse>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::torque_control >::value();
  }
  static const char* value(const ::motctrl_can::torque_controlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_TORQUE_CONTROL_H
