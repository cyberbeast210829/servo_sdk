// Generated by gencpp from file motctrl_can/reset_configuration.msg
// DO NOT EDIT!


#ifndef MOTCTRL_CAN_MESSAGE_RESET_CONFIGURATION_H
#define MOTCTRL_CAN_MESSAGE_RESET_CONFIGURATION_H

#include <ros/service_traits.h>


#include <motctrl_can/reset_configurationRequest.h>
#include <motctrl_can/reset_configurationResponse.h>


namespace motctrl_can
{

struct reset_configuration
{

typedef reset_configurationRequest Request;
typedef reset_configurationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct reset_configuration
} // namespace motctrl_can


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motctrl_can::reset_configuration > {
  static const char* value()
  {
    return "f4cf94077d46a7ac28e1686e63bb1b07";
  }

  static const char* value(const ::motctrl_can::reset_configuration&) { return value(); }
};

template<>
struct DataType< ::motctrl_can::reset_configuration > {
  static const char* value()
  {
    return "motctrl_can/reset_configuration";
  }

  static const char* value(const ::motctrl_can::reset_configuration&) { return value(); }
};


// service_traits::MD5Sum< ::motctrl_can::reset_configurationRequest> should match
// service_traits::MD5Sum< ::motctrl_can::reset_configuration >
template<>
struct MD5Sum< ::motctrl_can::reset_configurationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::reset_configuration >::value();
  }
  static const char* value(const ::motctrl_can::reset_configurationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::reset_configurationRequest> should match
// service_traits::DataType< ::motctrl_can::reset_configuration >
template<>
struct DataType< ::motctrl_can::reset_configurationRequest>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::reset_configuration >::value();
  }
  static const char* value(const ::motctrl_can::reset_configurationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motctrl_can::reset_configurationResponse> should match
// service_traits::MD5Sum< ::motctrl_can::reset_configuration >
template<>
struct MD5Sum< ::motctrl_can::reset_configurationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motctrl_can::reset_configuration >::value();
  }
  static const char* value(const ::motctrl_can::reset_configurationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motctrl_can::reset_configurationResponse> should match
// service_traits::DataType< ::motctrl_can::reset_configuration >
template<>
struct DataType< ::motctrl_can::reset_configurationResponse>
{
  static const char* value()
  {
    return DataType< ::motctrl_can::reset_configuration >::value();
  }
  static const char* value(const ::motctrl_can::reset_configurationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTCTRL_CAN_MESSAGE_RESET_CONFIGURATION_H