// Generated by gencpp from file motctrl_can/stop_motor.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_STOP_MOTOR_H
#define MOTCTRL_CAN_MESSAGE_STOP_MOTOR_H

#include <ros/service_traits.h>


#include <motctrl_can/stop_motorRequest.h>
#include <motctrl_can/stop_motorResponse.h>


namespace motctrl_can
{

struct stop_motor
{

typedef stop_motorRequest Request;
typedef stop_motorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct stop_motor
} // namespace motctrl_can


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motctrl_can::stop_motor > {
  static const char* value()
  {
    return "f4cf94077d46a7ac28e1686e63bb1b07";
  }

  static const char* value(const ::motctrl_can::stop_motor&) { return value(); }
};

template<>
struct DataType< ::motctrl_can::stop_motor > {
  static const char* value()
  {
    return "motctrl_can/stop_motor";
  }

  static const char* value(const ::motctrl_can::stop_motor&) { return value(); }
};


// service_traits::MD5Sum< ::motctrl_can::stop_motorRequest> should match
// service_traits::MD5Sum< ::motctrl_can::stop_motor >
template<>
struct MD5Sum< ::motctrl_can::stop_motorRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::stop_motor >::value();
  }
  static const char* value(const ::motctrl_can::stop_motorRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::stop_motorRequest> should match
// service_traits::DataType< ::motctrl_can::stop_motor >
template<>
struct DataType< ::motctrl_can::stop_motorRequest>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::stop_motor >::value();
  }
  static const char* value(const ::motctrl_can::stop_motorRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motctrl_can::stop_motorResponse> should match
// service_traits::MD5Sum< ::motctrl_can::stop_motor >
template<>
struct MD5Sum< ::motctrl_can::stop_motorResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::stop_motor >::value();
  }
  static const char* value(const ::motctrl_can::stop_motorResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::stop_motorResponse> should match
// service_traits::DataType< ::motctrl_can::stop_motor >
template<>
struct DataType< ::motctrl_can::stop_motorResponse>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::stop_motor >::value();
  }
  static const char* value(const ::motctrl_can::stop_motorResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_STOP_MOTOR_H