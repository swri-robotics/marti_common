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
#ifndef SWRI_ROUTE_UTIL_ROUTE_POINT_H_
#define SWRI_ROUTE_UTIL_ROUTE_POINT_H_

#include <string>
#include <map>

#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <marti_nav_msgs/msg/route_position.hpp>

namespace swri_route_util
{
// The RoutePoint class provides a more friendly interface for working
// with the marti_nav_msgs::Route and marti_nav_msgs::RoutePoint
// message types.  See comments for swri_route_util::RoutePoint for
// more information.  This class is used to represent both route
// points that are part of a Route, and independent points that were
// interpolated from a Route.
class RoutePoint
{
 public:
  RoutePoint();

  // Access to the route point's position as tf datatypes.
  void setPosition(const tf2::Vector3 &position);
  const tf2::Vector3& position() const;
  tf2::Vector3& position();

  // Access to the route point's position as message datatypes.
  void setPosition(const geometry_msgs::msg::Point &position);
  const geometry_msgs::msg::Point positionMsg() const;

  // Access to the route point's orientation as tf datatypes.
  void setOrientation(const tf2::Quaternion &orientation);
  const tf2::Quaternion& orientation() const;
  tf2::Quaternion& orientation();

  // Access to the route point's orientation as message datatypes.
  void setOrientation(const geometry_msgs::msg::Quaternion &orientation);
  const geometry_msgs::msg::Quaternion orientationMsg() const;

  // Access to the route point's pose (position and orientation) as
  // tf datatypes.
  //void setPose(const tf2::Pose &pose);
  //tf2::Pose pose() const;

  // Access to the route point's pose (position and orientation) as
  // message datatypes.
  void setPose(const geometry_msgs::msg::Pose &pose);
  geometry_msgs::msg::Pose poseMsg() const;

  // Access to the route point's id.  Ids should be unique when used,
  // but are typically not set for interpolated points.
  const std::string& id() const;
  void setId(const std::string &id);

  // Native access to the route point "stop_point" property.  This is
  // a boolean property that defaults to false if it is not explicitly
  // defined in the route.
  bool stopPoint() const;
  void setStopPoint(bool value);

  // Native access to the route point "stop_point_delay" property.
  // This is a floating point value that specifies how long the
  // vehicle should be paused at the specific point, in seconds.  The
  // delay defaults to 0.0 if it not defined in the route.
  double stopPointDelay() const;
  void setStopPointDelay(double delay);

  // Return a marti_nav_msgs::RoutePosition message that corresponds to this point.
  marti_nav_msgs::msg::RoutePosition routePosition() const;

  // The following methods provide general purpose access to route
  // point properties.  They will also correctly map to properties
  // that are exposed natively.  Native methods are generally faster
  // and safer and should be preferred when available.  Note: We have
  // not yet added any native properties for the route point.

  // Get a list of all the properties defined for this route point,
  // including native properties.
  std::vector<std::string> getPropertyNames() const;
  
  // Get the value of a property.  Returns an empty string if the
  // property does not exist.
  std::string getProperty(const std::string &name) const;

  // Determine if the specified property is defined for the route
  // point.
  bool hasProperty(const std::string &name) const;

  // Set the value of a property.  If the property doesn't exist, it
  // is added.
  void setProperty(const std::string &name, const std::string &value);

  // Delete a property.  If the property doesn't exist or is not
  // deletable (e.g. name, guid), this method does nothing.
  void deleteProperty(const std::string &name);
  
 private:
  tf2::Vector3 position_;
  tf2::Quaternion orientation_;

  std::string id_;

  bool stop_point_;
  double stop_point_delay_;

  std::map<std::string, std::string> properties_;
};  // class RoutePoint
} // namespace swri_route_util

#include "route_point_inline.h"

#endif  // SWRI_ROUTE_UTIL_ROUTE_POINT_H_
