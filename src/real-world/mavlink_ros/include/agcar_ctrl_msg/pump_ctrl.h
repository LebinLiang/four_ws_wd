// Generated by gencpp from file agcar_ctrl_msg/pump_ctrl.msg
// DO NOT EDIT!


#ifndef AGCAR_CTRL_MSG_MESSAGE_PUMP_CTRL_H
#define AGCAR_CTRL_MSG_MESSAGE_PUMP_CTRL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace agcar_ctrl_msg
{
template <class ContainerAllocator>
struct pump_ctrl_
{
  typedef pump_ctrl_<ContainerAllocator> Type;

  pump_ctrl_()
    : pump1_en(0)
    , pump2_en(0)  {
    }
  pump_ctrl_(const ContainerAllocator& _alloc)
    : pump1_en(0)
    , pump2_en(0)  {
  (void)_alloc;
    }



   typedef uint8_t _pump1_en_type;
  _pump1_en_type pump1_en;

   typedef uint8_t _pump2_en_type;
  _pump2_en_type pump2_en;





  typedef boost::shared_ptr< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> const> ConstPtr;

}; // struct pump_ctrl_

typedef ::agcar_ctrl_msg::pump_ctrl_<std::allocator<void> > pump_ctrl;

typedef boost::shared_ptr< ::agcar_ctrl_msg::pump_ctrl > pump_ctrlPtr;
typedef boost::shared_ptr< ::agcar_ctrl_msg::pump_ctrl const> pump_ctrlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator1> & lhs, const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator2> & rhs)
{
  return lhs.pump1_en == rhs.pump1_en &&
    lhs.pump2_en == rhs.pump2_en;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator1> & lhs, const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace agcar_ctrl_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8f2174192e75cee40c753ab1426eac29";
  }

  static const char* value(const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8f2174192e75cee4ULL;
  static const uint64_t static_value2 = 0x0c753ab1426eac29ULL;
};

template<class ContainerAllocator>
struct DataType< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "agcar_ctrl_msg/pump_ctrl";
  }

  static const char* value(const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 pump1_en\n"
"uint8 pump2_en\n"
;
  }

  static const char* value(const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pump1_en);
      stream.next(m.pump2_en);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pump_ctrl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::agcar_ctrl_msg::pump_ctrl_<ContainerAllocator>& v)
  {
    s << indent << "pump1_en: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump1_en);
    s << indent << "pump2_en: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pump2_en);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AGCAR_CTRL_MSG_MESSAGE_PUMP_CTRL_H
