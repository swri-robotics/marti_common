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
#ifndef SWRI_ROUTE_UTIL_ROUTE_POINT_INLINE_H_
#define SWRI_ROUTE_UTIL_ROUTE_POINT_INLINE_H_

#include <boost/lexical_cast.hpp>

namespace swri_route_util
{
inline
void RoutePoint::setPosition(const tf::Vector3 &position)
{
  position_ = position;
}

inline
const tf::Vector3& RoutePoint::position() const
{
  return position_;
}

inline
tf::Vector3& RoutePoint::position()
{
  return position_;
}

inline
void RoutePoint::setPosition(const geometry_msgs::Point &position)
{
  tf::pointMsgToTF(position, position_);
}

inline
const geometry_msgs::Point RoutePoint::positionMsg() const
{
  geometry_msgs::Point p;
  tf::pointTFToMsg(position_, p);
  return p;
}

inline
void RoutePoint::setOrientation(const tf::Quaternion &orientation)
{
  orientation_ = orientation;
}

inline
const tf::Quaternion& RoutePoint::orientation() const
{
  return orientation_;
}

inline
tf::Quaternion& RoutePoint::orientation()
{
  return orientation_;
}

inline
void RoutePoint::setOrientation(const geometry_msgs::Quaternion &orientation)
{
  tf::quaternionMsgToTF(orientation, orientation_);
}

inline
const geometry_msgs::Quaternion RoutePoint::orientationMsg() const
{
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(orientation_, q);
  return q;
}

inline
void RoutePoint::setPose(const tf::Pose &pose)
{
  setPosition(pose.getOrigin());
  setOrientation(pose.getRotation());
}

inline
tf::Pose RoutePoint::pose() const
{
  tf::Pose pose;
  pose.setOrigin(position_);
  pose.setRotation(orientation_);
  return pose;
}

inline
void RoutePoint::setPose(const geometry_msgs::Pose &pose)
{
  setPosition(pose.position);
  setOrientation(pose.orientation);
}

inline
geometry_msgs::Pose RoutePoint::poseMsg() const
{
  geometry_msgs::Pose pose;
  pose.position = positionMsg();
  pose.orientation = orientationMsg();
  return pose;
}
  
inline
const std::string& RoutePoint::id() const
{
  return id_;
}

inline
void RoutePoint::setId(const std::string &id)
{
  id_ = id;
}

inline
marti_nav_msgs::RoutePosition RoutePoint::routePosition() const
{
  marti_nav_msgs::RoutePosition position;
  position.id = id();
  position.distance = 0.0;
  return position;
}

inline
bool RoutePoint::stopPoint() const
{
  return stop_point_;
}

inline
void RoutePoint::setStopPoint(bool value)
{
  stop_point_ = value;
}

inline
double RoutePoint::stopPointDelay() const
{
  return stop_point_delay_;
}

inline
void RoutePoint::setStopPointDelay(double delay)
{
  stop_point_delay_ = delay;
}

template <typename T>
inline
T RoutePoint::getTypedProperty(const std::string &name) const
{
  return boost::lexical_cast<T>(getProperty(name));
}
} // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_ROUTE_POINT_INLINE_H_
