// Generated by gencpp from file framework/StrInt.msg
// DO NOT EDIT!


#ifndef FRAMEWORK_MESSAGE_STRINT_H
#define FRAMEWORK_MESSAGE_STRINT_H

#include <ros/service_traits.h>


#include <framework/StrIntRequest.h>
#include <framework/StrIntResponse.h>


namespace framework
{

struct StrInt
{

typedef StrIntRequest Request;
typedef StrIntResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StrInt
} // namespace framework


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::framework::StrInt > {
  static const char* value()
  {
    return "3df8c623795dfc3d210f796f50fd2d3c";
  }

  static const char* value(const ::framework::StrInt&) { return value(); }
};

template<>
struct DataType< ::framework::StrInt > {
  static const char* value()
  {
    return "framework/StrInt";
  }

  static const char* value(const ::framework::StrInt&) { return value(); }
};


// service_traits::MD5Sum< ::framework::StrIntRequest> should match 
// service_traits::MD5Sum< ::framework::StrInt > 
template<>
struct MD5Sum< ::framework::StrIntRequest>
{
  static const char* value()
  {
    return MD5Sum< ::framework::StrInt >::value();
  }
  static const char* value(const ::framework::StrIntRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::framework::StrIntRequest> should match 
// service_traits::DataType< ::framework::StrInt > 
template<>
struct DataType< ::framework::StrIntRequest>
{
  static const char* value()
  {
    return DataType< ::framework::StrInt >::value();
  }
  static const char* value(const ::framework::StrIntRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::framework::StrIntResponse> should match 
// service_traits::MD5Sum< ::framework::StrInt > 
template<>
struct MD5Sum< ::framework::StrIntResponse>
{
  static const char* value()
  {
    return MD5Sum< ::framework::StrInt >::value();
  }
  static const char* value(const ::framework::StrIntResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::framework::StrIntResponse> should match 
// service_traits::DataType< ::framework::StrInt > 
template<>
struct DataType< ::framework::StrIntResponse>
{
  static const char* value()
  {
    return DataType< ::framework::StrInt >::value();
  }
  static const char* value(const ::framework::StrIntResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FRAMEWORK_MESSAGE_STRINT_H
