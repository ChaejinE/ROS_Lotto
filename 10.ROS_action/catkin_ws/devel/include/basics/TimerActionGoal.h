// Generated by gencpp from file basics/TimerActionGoal.msg
// DO NOT EDIT!


#ifndef BASICS_MESSAGE_TIMERACTIONGOAL_H
#define BASICS_MESSAGE_TIMERACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <basics/TimerGoal.h>

namespace basics
{
template <class ContainerAllocator>
struct TimerActionGoal_
{
  typedef TimerActionGoal_<ContainerAllocator> Type;

  TimerActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  TimerActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::basics::TimerGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::basics::TimerActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::basics::TimerActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct TimerActionGoal_

typedef ::basics::TimerActionGoal_<std::allocator<void> > TimerActionGoal;

typedef boost::shared_ptr< ::basics::TimerActionGoal > TimerActionGoalPtr;
typedef boost::shared_ptr< ::basics::TimerActionGoal const> TimerActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::basics::TimerActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::basics::TimerActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::basics::TimerActionGoal_<ContainerAllocator1> & lhs, const ::basics::TimerActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::basics::TimerActionGoal_<ContainerAllocator1> & lhs, const ::basics::TimerActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace basics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::basics::TimerActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::basics::TimerActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::basics::TimerActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::basics::TimerActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::basics::TimerActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::basics::TimerActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::basics::TimerActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "db74ec180ecb81d0542047d87021844f";
  }

  static const char* value(const ::basics::TimerActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdb74ec180ecb81d0ULL;
  static const uint64_t static_value2 = 0x542047d87021844fULL;
};

template<class ContainerAllocator>
struct DataType< ::basics::TimerActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "basics/TimerActionGoal";
  }

  static const char* value(const ::basics::TimerActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::basics::TimerActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"TimerGoal goal\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: basics/TimerGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"duration time_to_wait\n"
;
  }

  static const char* value(const ::basics::TimerActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::basics::TimerActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimerActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::basics::TimerActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::basics::TimerActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::basics::TimerGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BASICS_MESSAGE_TIMERACTIONGOAL_H
