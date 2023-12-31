// Generated by gencpp from file motctrl_can/modify_parameter.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_MODIFY_PARAMETER_H
#define MOTCTRL_CAN_MESSAGE_MODIFY_PARAMETER_H

#include <ros/service_traits.h>


#include <motctrl_can/modify_parameterRequest.h>
#include <motctrl_can/modify_parameterResponse.h>


namespace motctrl_can
{

struct modify_parameter
{

typedef modify_parameterRequest Request;
typedef modify_parameterResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct modify_parameter
} // namespace motctrl_can


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motctrl_can::modify_parameter > {
  static const char* value()
  {
    return "a665fdb983aa987dfaef3dd71344e03e";
  }

  static const char* value(const ::motctrl_can::modify_parameter&) { return value(); }
};

template<>
struct DataType< ::motctrl_can::modify_parameter > {
  static const char* value()
  {
    return "motctrl_can/modify_parameter";
  }

  static const char* value(const ::motctrl_can::modify_parameter&) { return value(); }
};


// service_traits::MD5Sum< ::motctrl_can::modify_parameterRequest> should match
// service_traits::MD5Sum< ::motctrl_can::modify_parameter >
template<>
struct MD5Sum< ::motctrl_can::modify_parameterRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::modify_parameter >::value();
  }
  static const char* value(const ::motctrl_can::modify_parameterRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::modify_parameterRequest> should match
// service_traits::DataType< ::motctrl_can::modify_parameter >
template<>
struct DataType< ::motctrl_can::modify_parameterRequest>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::modify_parameter >::value();
  }
  static const char* value(const ::motctrl_can::modify_parameterRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motctrl_can::modify_parameterResponse> should match
// service_traits::MD5Sum< ::motctrl_can::modify_parameter >
template<>
struct MD5Sum< ::motctrl_can::modify_parameterResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::modify_parameter >::value();
  }
  static const char* value(const ::motctrl_can::modify_parameterResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::modify_parameterResponse> should match
// service_traits::DataType< ::motctrl_can::modify_parameter >
template<>
struct DataType< ::motctrl_can::modify_parameterResponse>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::modify_parameter >::value();
  }
  static const char* value(const ::motctrl_can::modify_parameterResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_MODIFY_PARAMETER_H
