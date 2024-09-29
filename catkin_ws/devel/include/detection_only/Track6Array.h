// Generated by gencpp from file detection_only/Track6Array.msg
// DO NOT EDIT!


#ifndef DETECTION_ONLY_MESSAGE_TRACK6ARRAY_H
#define DETECTION_ONLY_MESSAGE_TRACK6ARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <detection_only/Track_6.h>

namespace detection_only
{
template <class ContainerAllocator>
struct Track6Array_
{
  typedef Track6Array_<ContainerAllocator> Type;

  Track6Array_()
    : tracks()  {
    }
  Track6Array_(const ContainerAllocator& _alloc)
    : tracks(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::detection_only::Track_6_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::detection_only::Track_6_<ContainerAllocator> >> _tracks_type;
  _tracks_type tracks;





  typedef boost::shared_ptr< ::detection_only::Track6Array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::detection_only::Track6Array_<ContainerAllocator> const> ConstPtr;

}; // struct Track6Array_

typedef ::detection_only::Track6Array_<std::allocator<void> > Track6Array;

typedef boost::shared_ptr< ::detection_only::Track6Array > Track6ArrayPtr;
typedef boost::shared_ptr< ::detection_only::Track6Array const> Track6ArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::detection_only::Track6Array_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::detection_only::Track6Array_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::detection_only::Track6Array_<ContainerAllocator1> & lhs, const ::detection_only::Track6Array_<ContainerAllocator2> & rhs)
{
  return lhs.tracks == rhs.tracks;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::detection_only::Track6Array_<ContainerAllocator1> & lhs, const ::detection_only::Track6Array_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace detection_only

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::detection_only::Track6Array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::detection_only::Track6Array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_only::Track6Array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_only::Track6Array_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_only::Track6Array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_only::Track6Array_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::detection_only::Track6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bcae8edf8c2ae4b9b6e115eb41e70bec";
  }

  static const char* value(const ::detection_only::Track6Array_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbcae8edf8c2ae4b9ULL;
  static const uint64_t static_value2 = 0xb6e115eb41e70becULL;
};

template<class ContainerAllocator>
struct DataType< ::detection_only::Track6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "detection_only/Track6Array";
  }

  static const char* value(const ::detection_only::Track6Array_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::detection_only::Track6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# bboxes \n"
"# [x1,y1,x2,y2,idx,class]\n"
"Track_6[] tracks \n"
"================================================================================\n"
"MSG: detection_only/Track_6\n"
"# 1 bbox \n"
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

  static const char* value(const ::detection_only::Track6Array_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::detection_only::Track6Array_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tracks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Track6Array_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::detection_only::Track6Array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::detection_only::Track6Array_<ContainerAllocator>& v)
  {
    s << indent << "tracks[]" << std::endl;
    for (size_t i = 0; i < v.tracks.size(); ++i)
    {
      s << indent << "  tracks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::detection_only::Track_6_<ContainerAllocator> >::stream(s, indent + "    ", v.tracks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DETECTION_ONLY_MESSAGE_TRACK6ARRAY_H
