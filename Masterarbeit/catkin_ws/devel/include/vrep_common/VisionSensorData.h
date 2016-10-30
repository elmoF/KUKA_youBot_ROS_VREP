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
 * Auto-generated by genmsg_cpp from file /home/youbot/Simulation_Examples/Masterarbeit/catkin_ws/src/vrep_common/msg/VisionSensorData.msg
 *
 */


#ifndef VREP_COMMON_MESSAGE_VISIONSENSORDATA_H
#define VREP_COMMON_MESSAGE_VISIONSENSORDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

namespace vrep_common
{
template <class ContainerAllocator>
struct VisionSensorData_
{
  typedef VisionSensorData_<ContainerAllocator> Type;

  VisionSensorData_()
    : triggerState()
    , packetData()
    , packetSizes()  {
    }
  VisionSensorData_(const ContainerAllocator& _alloc)
    : triggerState(_alloc)
    , packetData(_alloc)
    , packetSizes(_alloc)  {
    }



   typedef  ::std_msgs::Int32_<ContainerAllocator>  _triggerState_type;
  _triggerState_type triggerState;

   typedef  ::std_msgs::Float32MultiArray_<ContainerAllocator>  _packetData_type;
  _packetData_type packetData;

   typedef  ::std_msgs::Int32MultiArray_<ContainerAllocator>  _packetSizes_type;
  _packetSizes_type packetSizes;




  typedef boost::shared_ptr< ::vrep_common::VisionSensorData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::VisionSensorData_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct VisionSensorData_

typedef ::vrep_common::VisionSensorData_<std::allocator<void> > VisionSensorData;

typedef boost::shared_ptr< ::vrep_common::VisionSensorData > VisionSensorDataPtr;
typedef boost::shared_ptr< ::vrep_common::VisionSensorData const> VisionSensorDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrep_common::VisionSensorData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrep_common::VisionSensorData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vrep_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/hydro/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'vrep_common': ['/home/youbot/Simulation_Examples/Masterarbeit/catkin_ws/src/vrep_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::VisionSensorData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::VisionSensorData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::VisionSensorData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::VisionSensorData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::VisionSensorData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::VisionSensorData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::VisionSensorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f2ef61369c5dd779545031da241014b8";
  }

  static const char* value(const ::vrep_common::VisionSensorData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf2ef61369c5dd779ULL;
  static const uint64_t static_value2 = 0x545031da241014b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::VisionSensorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrep_common/VisionSensorData";
  }

  static const char* value(const ::vrep_common::VisionSensorData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::VisionSensorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Int32 triggerState\n\
std_msgs/Float32MultiArray packetData\n\
std_msgs/Int32MultiArray packetSizes\n\
\n\
================================================================================\n\
MSG: std_msgs/Int32\n\
int32 data\n\
================================================================================\n\
MSG: std_msgs/Float32MultiArray\n\
# Please look at the MultiArrayLayout message definition for\n\
# documentation on all multiarrays.\n\
\n\
MultiArrayLayout  layout        # specification of data layout\n\
float32[]         data          # array of data\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/MultiArrayLayout\n\
# The multiarray declares a generic multi-dimensional array of a\n\
# particular data type.  Dimensions are ordered from outer most\n\
# to inner most.\n\
\n\
MultiArrayDimension[] dim # Array of dimension properties\n\
uint32 data_offset        # padding bytes at front of data\n\
\n\
# Accessors should ALWAYS be written in terms of dimension stride\n\
# and specified outer-most dimension first.\n\
# \n\
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n\
#\n\
# A standard, 3-channel 640x480 image with interleaved color channels\n\
# would be specified as:\n\
#\n\
# dim[0].label  = \"height\"\n\
# dim[0].size   = 480\n\
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n\
# dim[1].label  = \"width\"\n\
# dim[1].size   = 640\n\
# dim[1].stride = 3*640 = 1920\n\
# dim[2].label  = \"channel\"\n\
# dim[2].size   = 3\n\
# dim[2].stride = 3\n\
#\n\
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\
================================================================================\n\
MSG: std_msgs/MultiArrayDimension\n\
string label   # label of given dimension\n\
uint32 size    # size of given dimension (in type units)\n\
uint32 stride  # stride of given dimension\n\
================================================================================\n\
MSG: std_msgs/Int32MultiArray\n\
# Please look at the MultiArrayLayout message definition for\n\
# documentation on all multiarrays.\n\
\n\
MultiArrayLayout  layout        # specification of data layout\n\
int32[]           data          # array of data\n\
\n\
";
  }

  static const char* value(const ::vrep_common::VisionSensorData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrep_common::VisionSensorData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.triggerState);
      stream.next(m.packetData);
      stream.next(m.packetSizes);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct VisionSensorData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrep_common::VisionSensorData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrep_common::VisionSensorData_<ContainerAllocator>& v)
  {
    s << indent << "triggerState: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.triggerState);
    s << indent << "packetData: ";
    s << std::endl;
    Printer< ::std_msgs::Float32MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.packetData);
    s << indent << "packetSizes: ";
    s << std::endl;
    Printer< ::std_msgs::Int32MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.packetSizes);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VREP_COMMON_MESSAGE_VISIONSENSORDATA_H
