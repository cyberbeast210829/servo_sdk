// Generated by gencpp from file motctrl_can/position_control.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_POSITION_CONTROL_H
#define MOTCTRL_CAN_MESSAGE_POSITION_CONTROL_H

#include <ros/service_traits.h>


#include <motctrl_can/position_controlRequest.h>
#include <motctrl_can/position_controlResponse.h>


namespace motctrl_can
{

struct position_control
{

typedef position_controlRequest Request;
typedef position_controlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct position_control
} // namespace motctrl_can


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motctrl_can::position_control > {
  static const char* value()
  {
    return "cc6082d378d8d628dc1e3db3d315d172";
  }

  static const char* value(const ::motctrl_can::position_control&) { return value(); }
};

template<>
struct DataType< ::motctrl_can::position_control > {
  static const char* value()
  {
    return "motctrl_can/position_control";
  }

  static const char* value(const ::motctrl_can::position_control&) { return value(); }
};


// service_traits::MD5Sum< ::motctrl_can::position_controlRequest> should match
// service_traits::MD5Sum< ::motctrl_can::position_control >
template<>
struct MD5Sum< ::motctrl_can::position_controlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::position_control >::value();
  }
  static const char* value(const ::motctrl_can::position_controlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::position_controlRequest> should match
// service_traits::DataType< ::motctrl_can::position_control >
template<>
struct DataType< ::motctrl_can::position_controlRequest>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::position_control >::value();
  }
  static const char* value(const ::motctrl_can::position_controlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motctrl_can::position_controlResponse> should match
// service_traits::MD5Sum< ::motctrl_can::position_control >
template<>
struct MD5Sum< ::motctrl_can::position_controlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::position_control >::value();
  }
  static const char* value(const ::motctrl_can::position_controlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::position_controlResponse> should match
// service_traits::DataType< ::motctrl_can::position_control >
template<>
struct DataType< ::motctrl_can::position_controlResponse>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::position_control >::value();
  }
  static const char* value(const ::motctrl_can::position_controlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_POSITION_CONTROL_H
