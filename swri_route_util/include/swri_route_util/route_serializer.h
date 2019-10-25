// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

// TODO pjr I'm not even sure what to do with this for ROS2...

#include <ros/message_traits.h>
#include <ros/serialization.h>


namespace ros
{
namespace message_traits
{
// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
// http://wiki.ros.org/roscpp/Overview/MessagesTraits

// This type is not fixed-size.
template<> struct IsFixedSize<swri_route_util::Route> : public FalseType {};
// This type is memcpyable
template<> struct IsSimple<swri_route_util::Route> : public TrueType {};
// This type has a header
template<> struct HasHeader<swri_route_util::Route> : public TrueType {};

template<>
struct MD5Sum<swri_route_util::Route>
{
  static const char* value()
  {
    // Ensure that if the definition of marti_nav_msgs::Route changes
    // we have a compile error here.
    ROS_STATIC_ASSERT(MD5Sum<marti_nav_msgs::Route>::static_value1 == 0x626dfe06202116afULL);
    ROS_STATIC_ASSERT(MD5Sum<marti_nav_msgs::Route>::static_value2 == 0xac99e6de9fa42b3eULL);
    return MD5Sum<marti_nav_msgs::Route>::value();
  }

  static const char* value(const swri_route_util::Route& m)
  {
    return MD5Sum<swri_route_util::Route>::value();
  }
};  // struct MD5Sum<swri_route_util::Route>

template<>
struct DataType<swri_route_util::Route>
{
  static const char* value()
  {
    return DataType<marti_nav_msgs::Route>::value();
  }

  static const char* value(const swri_route_util::Route& m)
  {
    return DataType<swri_route_util::Route>::value();
  }
};  // struct DataType<swri_route_util::Route>

template<>
struct Definition<swri_route_util::Route>
{
  static const char* value()
  {
    return Definition<marti_nav_msgs::Route>::value();
  }

  static const char* value(const swri_route_util::Route& m)
  {
    return Definition<swri_route_util::Route>::value();
  }
};  // struct Definition<swri_route_util::Route>
}  // namespace message_traits

namespace serialization
{
template<>
struct Serializer<swri_route_util::Route>
{
  template<typename Stream>
  inline static void write(Stream& stream, const swri_route_util::Route& route)
  {
    marti_nav_msgs::Route msg;
    route.toMsg(msg);
    stream.next(msg);
  }

  template<typename Stream>
  inline static void read(Stream& stream, swri_route_util::Route& route)
  {
    marti_nav_msgs::Route msg;
    stream.next(msg);
    route = swri_route_util::Route(msg);
  }

  inline static uint32_t serializedLength(const swri_route_util::Route& route)
  {
    marti_nav_msgs::Route msg;
    route.toMsg(msg);
    return serializationLength(msg);
  }
};
}  // namespace serialization
}  // namespace ros
