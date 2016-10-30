/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/youbot/Simulation_Examples/towerOfHanoi_ROS/catkin_ws/src/vrep_common/srv/simRosCallScriptFunction.srv
 *
 */


#ifndef VREP_COMMON_MESSAGE_SIMROSCALLSCRIPTFUNCTIONREQUEST_H
#define VREP_COMMON_MESSAGE_SIMROSCALLSCRIPTFUNCTIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vrep_common
{
template <class ContainerAllocator>
struct simRosCallScriptFunctionRequest_
{
  typedef simRosCallScriptFunctionRequest_<ContainerAllocator> Type;

  simRosCallScriptFunctionRequest_()
    : functionNameAtObjectName()
    , scriptHandleOrType(0)
    , inputInts()
    , inputFloats()
    , inputStrings()
    , inputBuffer()  {
    }
  simRosCallScriptFunctionRequest_(const ContainerAllocator& _alloc)
    : functionNameAtObjectName(_alloc)
    , scriptHandleOrType(0)
    , inputInts(_alloc)
    , inputFloats(_alloc)
    , inputStrings(_alloc)
    , inputBuffer(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _functionNameAtObjectName_type;
  _functionNameAtObjectName_type functionNameAtObjectName;

   typedef int32_t _scriptHandleOrType_type;
  _scriptHandleOrType_type scriptHandleOrType;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _inputInts_type;
  _inputInts_type inputInts;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _inputFloats_type;
  _inputFloats_type inputFloats;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _inputStrings_type;
  _inputStrings_type inputStrings;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _inputBuffer_type;
  _inputBuffer_type inputBuffer;




  typedef boost::shared_ptr< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct simRosCallScriptFunctionRequest_

typedef ::vrep_common::simRosCallScriptFunctionRequest_<std::allocator<void> > simRosCallScriptFunctionRequest;

typedef boost::shared_ptr< ::vrep_common::simRosCallScriptFunctionRequest > simRosCallScriptFunctionRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosCallScriptFunctionRequest const> simRosCallScriptFunctionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vrep_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/hydro/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'vrep_common': ['/home/youbot/Simulation_Examples/towerOfHanoi_ROS/catkin_ws/src/vrep_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e13770ffacc84bd3ba12cae1ce2b7f0e";
  }

  static const char* value(const ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe13770ffacc84bd3ULL;
  static const uint64_t static_value2 = 0xba12cae1ce2b7f0eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrep_common/simRosCallScriptFunctionRequest";
  }

  static const char* value(const ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
string functionNameAtObjectName\n\
int32 scriptHandleOrType\n\
int32[] inputInts\n\
float32[] inputFloats\n\
string[] inputStrings\n\
string inputBuffer\n\
";
  }

  static const char* value(const ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.functionNameAtObjectName);
      stream.next(m.scriptHandleOrType);
      stream.next(m.inputInts);
      stream.next(m.inputFloats);
      stream.next(m.inputStrings);
      stream.next(m.inputBuffer);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct simRosCallScriptFunctionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrep_common::simRosCallScriptFunctionRequest_<ContainerAllocator>& v)
  {
    s << indent << "functionNameAtObjectName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.functionNameAtObjectName);
    s << indent << "scriptHandleOrType: ";
    Printer<int32_t>::stream(s, indent + "  ", v.scriptHandleOrType);
    s << indent << "inputInts[]" << std::endl;
    for (size_t i = 0; i < v.inputInts.size(); ++i)
    {
      s << indent << "  inputInts[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.inputInts[i]);
    }
    s << indent << "inputFloats[]" << std::endl;
    for (size_t i = 0; i < v.inputFloats.size(); ++i)
    {
      s << indent << "  inputFloats[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.inputFloats[i]);
    }
    s << indent << "inputStrings[]" << std::endl;
    for (size_t i = 0; i < v.inputStrings.size(); ++i)
    {
      s << indent << "  inputStrings[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.inputStrings[i]);
    }
    s << indent << "inputBuffer: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.inputBuffer);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSCALLSCRIPTFUNCTIONREQUEST_H
