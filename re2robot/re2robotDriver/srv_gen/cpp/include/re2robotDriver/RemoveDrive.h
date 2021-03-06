/* Auto-generated by genmsg_cpp for file /home/dave/ros/re2robot/re2robotDriver/srv/RemoveDrive.srv */
#ifndef RE2ROBOTDRIVER_SERVICE_REMOVEDRIVE_H
#define RE2ROBOTDRIVER_SERVICE_REMOVEDRIVE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace re2robotDriver
{
template <class ContainerAllocator>
struct RemoveDriveRequest_ : public ros::Message
{
  typedef RemoveDriveRequest_<ContainerAllocator> Type;

  RemoveDriveRequest_()
  : name()
  {
  }

  RemoveDriveRequest_(const ContainerAllocator& _alloc)
  : name(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;


private:
  static const char* __s_getDataType_() { return "re2robotDriver/RemoveDriveRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "c1f3d28f1b044c871e6eff2e9fc3c667"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "d08a3b641c2f8680fbdfb1ea2e17a3e1"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string name\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, name);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, name);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(name);
    return size;
  }

  typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct RemoveDriveRequest
typedef  ::re2robotDriver::RemoveDriveRequest_<std::allocator<void> > RemoveDriveRequest;

typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveRequest> RemoveDriveRequestPtr;
typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveRequest const> RemoveDriveRequestConstPtr;


template <class ContainerAllocator>
struct RemoveDriveResponse_ : public ros::Message
{
  typedef RemoveDriveResponse_<ContainerAllocator> Type;

  RemoveDriveResponse_()
  : success(false)
  {
  }

  RemoveDriveResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


private:
  static const char* __s_getDataType_() { return "re2robotDriver/RemoveDriveResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "358e233cde0c8a8bcfea4ce193f8fc15"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "d08a3b641c2f8680fbdfb1ea2e17a3e1"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool success\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, success);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, success);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(success);
    return size;
  }

  typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct RemoveDriveResponse
typedef  ::re2robotDriver::RemoveDriveResponse_<std::allocator<void> > RemoveDriveResponse;

typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveResponse> RemoveDriveResponsePtr;
typedef boost::shared_ptr< ::re2robotDriver::RemoveDriveResponse const> RemoveDriveResponseConstPtr;

struct RemoveDrive
{

typedef RemoveDriveRequest Request;
typedef RemoveDriveResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct RemoveDrive
} // namespace re2robotDriver

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re2robotDriver/RemoveDriveRequest";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
\n\
";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re2robotDriver/RemoveDriveResponse";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
";
  }

  static const char* value(const  ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::re2robotDriver::RemoveDriveRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RemoveDriveRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::re2robotDriver::RemoveDriveResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RemoveDriveResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<re2robotDriver::RemoveDrive> {
  static const char* value() 
  {
    return "d08a3b641c2f8680fbdfb1ea2e17a3e1";
  }

  static const char* value(const re2robotDriver::RemoveDrive&) { return value(); } 
};

template<>
struct DataType<re2robotDriver::RemoveDrive> {
  static const char* value() 
  {
    return "re2robotDriver/RemoveDrive";
  }

  static const char* value(const re2robotDriver::RemoveDrive&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d08a3b641c2f8680fbdfb1ea2e17a3e1";
  }

  static const char* value(const re2robotDriver::RemoveDriveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<re2robotDriver::RemoveDriveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re2robotDriver/RemoveDrive";
  }

  static const char* value(const re2robotDriver::RemoveDriveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d08a3b641c2f8680fbdfb1ea2e17a3e1";
  }

  static const char* value(const re2robotDriver::RemoveDriveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<re2robotDriver::RemoveDriveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re2robotDriver/RemoveDrive";
  }

  static const char* value(const re2robotDriver::RemoveDriveResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RE2ROBOTDRIVER_SERVICE_REMOVEDRIVE_H

