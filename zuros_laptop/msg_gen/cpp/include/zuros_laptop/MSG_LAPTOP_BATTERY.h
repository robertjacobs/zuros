/* Auto-generated by genmsg_cpp for file /home/robot/git/zuros/zuros_laptop/msg/MSG_LAPTOP_BATTERY.msg */
#ifndef ZUROS_LAPTOP_MESSAGE_MSG_LAPTOP_BATTERY_H
#define ZUROS_LAPTOP_MESSAGE_MSG_LAPTOP_BATTERY_H
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


namespace zuros_laptop
{
template <class ContainerAllocator>
struct MSG_LAPTOP_BATTERY_ {
  typedef MSG_LAPTOP_BATTERY_<ContainerAllocator> Type;

  MSG_LAPTOP_BATTERY_()
  : battery_name()
  , state()
  , percentage()
  , remaining()
  {
  }

  MSG_LAPTOP_BATTERY_(const ContainerAllocator& _alloc)
  : battery_name(_alloc)
  , state(_alloc)
  , percentage(_alloc)
  , remaining(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _battery_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  battery_name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _state_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  state;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _percentage_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  percentage;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _remaining_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  remaining;


  typedef boost::shared_ptr< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MSG_LAPTOP_BATTERY
typedef  ::zuros_laptop::MSG_LAPTOP_BATTERY_<std::allocator<void> > MSG_LAPTOP_BATTERY;

typedef boost::shared_ptr< ::zuros_laptop::MSG_LAPTOP_BATTERY> MSG_LAPTOP_BATTERYPtr;
typedef boost::shared_ptr< ::zuros_laptop::MSG_LAPTOP_BATTERY const> MSG_LAPTOP_BATTERYConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace zuros_laptop

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d625544efcdab63ea31b1c72901c2d10";
  }

  static const char* value(const  ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd625544efcdab63eULL;
  static const uint64_t static_value2 = 0xa31b1c72901c2d10ULL;
};

template<class ContainerAllocator>
struct DataType< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> > {
  static const char* value() 
  {
    return "zuros_laptop/MSG_LAPTOP_BATTERY";
  }

  static const char* value(const  ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string battery_name\n\
string state\n\
string percentage\n\
string remaining\n\
\n\
";
  }

  static const char* value(const  ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.battery_name);
    stream.next(m.state);
    stream.next(m.percentage);
    stream.next(m.remaining);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MSG_LAPTOP_BATTERY_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::zuros_laptop::MSG_LAPTOP_BATTERY_<ContainerAllocator> & v) 
  {
    s << indent << "battery_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.battery_name);
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state);
    s << indent << "percentage: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.percentage);
    s << indent << "remaining: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.remaining);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ZUROS_LAPTOP_MESSAGE_MSG_LAPTOP_BATTERY_H

