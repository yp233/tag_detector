// Generated by gencpp from file tag_detector/ServiceMsgRequest.msg
// DO NOT EDIT!


#ifndef TAG_DETECTOR_MESSAGE_SERVICEMSGREQUEST_H
#define TAG_DETECTOR_MESSAGE_SERVICEMSGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tag_detector
{
template <class ContainerAllocator>
struct ServiceMsgRequest_
{
  typedef ServiceMsgRequest_<ContainerAllocator> Type;

  ServiceMsgRequest_()
    : size1(0)
    , size2(0)  {
    }
  ServiceMsgRequest_(const ContainerAllocator& _alloc)
    : size1(0)
    , size2(0)  {
  (void)_alloc;
    }



   typedef int32_t _size1_type;
  _size1_type size1;

   typedef int32_t _size2_type;
  _size2_type size2;





  typedef boost::shared_ptr< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ServiceMsgRequest_

typedef ::tag_detector::ServiceMsgRequest_<std::allocator<void> > ServiceMsgRequest;

typedef boost::shared_ptr< ::tag_detector::ServiceMsgRequest > ServiceMsgRequestPtr;
typedef boost::shared_ptr< ::tag_detector::ServiceMsgRequest const> ServiceMsgRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tag_detector::ServiceMsgRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tag_detector

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c63cae4a9bc1989d6d22ddaf34a9e93b";
  }

  static const char* value(const ::tag_detector::ServiceMsgRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc63cae4a9bc1989dULL;
  static const uint64_t static_value2 = 0x6d22ddaf34a9e93bULL;
};

template<class ContainerAllocator>
struct DataType< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tag_detector/ServiceMsgRequest";
  }

  static const char* value(const ::tag_detector::ServiceMsgRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 size1\n\
int32 size2\n\
";
  }

  static const char* value(const ::tag_detector::ServiceMsgRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.size1);
      stream.next(m.size2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ServiceMsgRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tag_detector::ServiceMsgRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tag_detector::ServiceMsgRequest_<ContainerAllocator>& v)
  {
    s << indent << "size1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.size1);
    s << indent << "size2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.size2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TAG_DETECTOR_MESSAGE_SERVICEMSGREQUEST_H