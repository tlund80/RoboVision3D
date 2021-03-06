/* Auto-generated by genmsg_cpp for file /home/jeppe/workspace-d3/robovision3d/host/vrm3dvision/srv/setGain.srv */
#ifndef VRM3DVISION_SERVICE_SETGAIN_H
#define VRM3DVISION_SERVICE_SETGAIN_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace vrm3dvision
{
template <class ContainerAllocator>
struct setGainRequest_ {
  typedef setGainRequest_<ContainerAllocator> Type;

  setGainRequest_()
  : camera(0)
  , gain(0.0)
  {
  }

  setGainRequest_(const ContainerAllocator& _alloc)
  : camera(0)
  , gain(0.0)
  {
  }

  typedef int8_t _camera_type;
  int8_t camera;

  typedef float _gain_type;
  float gain;

  enum { LEFT_CAMERA = 1 };
  enum { RIGHT_CAMERA = 2 };
  enum { COLOR_CAMERA = 4 };

  typedef boost::shared_ptr< ::vrm3dvision::setGainRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrm3dvision::setGainRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct setGainRequest
typedef  ::vrm3dvision::setGainRequest_<std::allocator<void> > setGainRequest;

typedef boost::shared_ptr< ::vrm3dvision::setGainRequest> setGainRequestPtr;
typedef boost::shared_ptr< ::vrm3dvision::setGainRequest const> setGainRequestConstPtr;



template <class ContainerAllocator>
struct setGainResponse_ {
  typedef setGainResponse_<ContainerAllocator> Type;

  setGainResponse_()
  : success(false)
  {
  }

  setGainResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::vrm3dvision::setGainResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrm3dvision::setGainResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct setGainResponse
typedef  ::vrm3dvision::setGainResponse_<std::allocator<void> > setGainResponse;

typedef boost::shared_ptr< ::vrm3dvision::setGainResponse> setGainResponsePtr;
typedef boost::shared_ptr< ::vrm3dvision::setGainResponse const> setGainResponseConstPtr;


struct setGain
{

typedef setGainRequest Request;
typedef setGainResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct setGain
} // namespace vrm3dvision

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrm3dvision::setGainRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrm3dvision::setGainRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrm3dvision::setGainRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8fe6acca52db242e9371361b287521c2";
  }

  static const char* value(const  ::vrm3dvision::setGainRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8fe6acca52db242eULL;
  static const uint64_t static_value2 = 0x9371361b287521c2ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrm3dvision::setGainRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrm3dvision/setGainRequest";
  }

  static const char* value(const  ::vrm3dvision::setGainRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrm3dvision::setGainRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 LEFT_CAMERA = 1\n\
int8 RIGHT_CAMERA = 2\n\
int8 COLOR_CAMERA = 4\n\
\n\
int8 camera\n\
\n\
float32 gain\n\
\n\
";
  }

  static const char* value(const  ::vrm3dvision::setGainRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrm3dvision::setGainRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrm3dvision::setGainResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrm3dvision::setGainResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrm3dvision::setGainResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::vrm3dvision::setGainResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrm3dvision::setGainResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrm3dvision/setGainResponse";
  }

  static const char* value(const  ::vrm3dvision::setGainResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrm3dvision::setGainResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrm3dvision::setGainResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrm3dvision::setGainResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrm3dvision::setGainRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.camera);
    stream.next(m.gain);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct setGainRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrm3dvision::setGainResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct setGainResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrm3dvision::setGain> {
  static const char* value() 
  {
    return "ed8bcecf8b13e55a92dc371339e8422c";
  }

  static const char* value(const vrm3dvision::setGain&) { return value(); } 
};

template<>
struct DataType<vrm3dvision::setGain> {
  static const char* value() 
  {
    return "vrm3dvision/setGain";
  }

  static const char* value(const vrm3dvision::setGain&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrm3dvision::setGainRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ed8bcecf8b13e55a92dc371339e8422c";
  }

  static const char* value(const vrm3dvision::setGainRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrm3dvision::setGainRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrm3dvision/setGain";
  }

  static const char* value(const vrm3dvision::setGainRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrm3dvision::setGainResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ed8bcecf8b13e55a92dc371339e8422c";
  }

  static const char* value(const vrm3dvision::setGainResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrm3dvision::setGainResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrm3dvision/setGain";
  }

  static const char* value(const vrm3dvision::setGainResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VRM3DVISION_SERVICE_SETGAIN_H

