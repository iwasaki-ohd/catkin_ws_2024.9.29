// Generated by gencpp from file detection_only/Track_6.msg
// DO NOT EDIT!


#ifndef DETECTION_ONLY_MESSAGE_TRACK_6_H
#define DETECTION_ONLY_MESSAGE_TRACK_6_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace detection_only
{
template <class ContainerAllocator>
struct Track_6_
{
  typedef Track_6_<ContainerAllocator> Type;

  Track_6_()
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)
    , id(0.0)
    , cls(0.0)  {
    }
  Track_6_(const ContainerAllocator& _alloc)
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)
    , id(0.0)
    , cls(0.0)  {
  (void)_alloc;
    }



   typedef float _x1_type;
  _x1_type x1;

   typedef float _y1_type;
  _y1_type y1;

   typedef float _x2_type;
  _x2_type x2;

   typedef float _y2_type;
  _y2_type y2;

   typedef float _id_type;
  _id_type id;

   typedef float _cls_type;
  _cls_type cls;





  typedef boost::shared_ptr< ::detection_only::Track_6_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::detection_only::Track_6_<ContainerAllocator> const> ConstPtr;

}; // struct Track_6_

typedef ::detection_only::Track_6_<std::allocator<void> > Track_6;

typedef boost::shared_ptr< ::detection_only::Track_6 > Track_6Ptr;
typedef boost::shared_ptr< ::detection_only::Track_6 const> Track_6ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::detection_only::Track_6_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::detection_only::Track_6_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::detection_only::Track_6_<ContainerAllocator1> & lhs, const ::detection_only::Track_6_<ContainerAllocator2> & rhs)
{
  return lhs.x1 == rhs.x1 &&
    lhs.y1 == rhs.y1 &&
    lhs.x2 == rhs.x2 &&
    lhs.y2 == rhs.y2 &&
    lhs.id == rhs.id &&
    lhs.cls == rhs.cls;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::detection_only::Track_6_<ContainerAllocator1> & lhs, const ::detection_only::Track_6_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace detection_only

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::detection_only::Track_6_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::detection_only::Track_6_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_only::Track_6_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_only::Track_6_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_only::Track_6_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_only::Track_6_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::detection_only::Track_6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "96df40b26fc827e6741cb22e1a845839";
  }

  static const char* value(const ::detection_only::Track_6_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x96df40b26fc827e6ULL;
  static const uint64_t static_value2 = 0x741cb22e1a845839ULL;
};

template<class ContainerAllocator>
struct DataType< ::detection_only::Track_6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "detection_only/Track_6";
  }

  static const char* value(const ::detection_only::Track_6_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::detection_only::Track_6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 1 bbox \n"
"# [x1,y1,x2,y2,conf,class]\n"
"# float32[] bbox_info\n"
"float32 x1\n"
"float32 y1\n"
"float32 x2\n"
"float32 y2\n"
"float32 id\n"
"float32 cls\n"
;
  }

  static const char* value(const ::detection_only::Track_6_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::detection_only::Track_6_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.y1);
      stream.next(m.x2);
      stream.next(m.y2);
      stream.next(m.id);
      stream.next(m.cls);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Track_6_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::detection_only::Track_6_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::detection_only::Track_6_<ContainerAllocator>& v)
  {
    s << indent << "x1: ";
    Printer<float>::stream(s, indent + "  ", v.x1);
    s << indent << "y1: ";
    Printer<float>::stream(s, indent + "  ", v.y1);
    s << indent << "x2: ";
    Printer<float>::stream(s, indent + "  ", v.x2);
    s << indent << "y2: ";
    Printer<float>::stream(s, indent + "  ", v.y2);
    s << indent << "id: ";
    Printer<float>::stream(s, indent + "  ", v.id);
    s << indent << "cls: ";
    Printer<float>::stream(s, indent + "  ", v.cls);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DETECTION_ONLY_MESSAGE_TRACK_6_H
