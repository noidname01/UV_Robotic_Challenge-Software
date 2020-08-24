// Generated by gencpp from file practice/my_srvResponse.msg
// DO NOT EDIT!


#ifndef PRACTICE_MESSAGE_MY_SRVRESPONSE_H
#define PRACTICE_MESSAGE_MY_SRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace practice
{
template <class ContainerAllocator>
struct my_srvResponse_
{
  typedef my_srvResponse_<ContainerAllocator> Type;

  my_srvResponse_()
    : name()
    , gender()
    , age(0)  {
    }
  my_srvResponse_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , gender(_alloc)
    , age(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _gender_type;
  _gender_type gender;

   typedef int64_t _age_type;
  _age_type age;





  typedef boost::shared_ptr< ::practice::my_srvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::practice::my_srvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct my_srvResponse_

typedef ::practice::my_srvResponse_<std::allocator<void> > my_srvResponse;

typedef boost::shared_ptr< ::practice::my_srvResponse > my_srvResponsePtr;
typedef boost::shared_ptr< ::practice::my_srvResponse const> my_srvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::practice::my_srvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::practice::my_srvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::practice::my_srvResponse_<ContainerAllocator1> & lhs, const ::practice::my_srvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.gender == rhs.gender &&
    lhs.age == rhs.age;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::practice::my_srvResponse_<ContainerAllocator1> & lhs, const ::practice::my_srvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace practice

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::practice::my_srvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::practice::my_srvResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::practice::my_srvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::practice::my_srvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::practice::my_srvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::practice::my_srvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::practice::my_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76c521bbfe60b764a6e5c48ba35bd8f9";
  }

  static const char* value(const ::practice::my_srvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76c521bbfe60b764ULL;
  static const uint64_t static_value2 = 0xa6e5c48ba35bd8f9ULL;
};

template<class ContainerAllocator>
struct DataType< ::practice::my_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "practice/my_srvResponse";
  }

  static const char* value(const ::practice::my_srvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::practice::my_srvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"string gender\n"
"int64 age\n"
"\n"
;
  }

  static const char* value(const ::practice::my_srvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::practice::my_srvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.gender);
      stream.next(m.age);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct my_srvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::practice::my_srvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::practice::my_srvResponse_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "gender: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.gender);
    s << indent << "age: ";
    Printer<int64_t>::stream(s, indent + "  ", v.age);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PRACTICE_MESSAGE_MY_SRVRESPONSE_H