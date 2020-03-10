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
#ifndef SWRI_ROUTE_UTIL_ROUTE_H_
#define SWRI_ROUTE_UTIL_ROUTE_H_

#include <string>
#include <map>
#include <vector>

#include <boost/lexical_cast.hpp>

#include <marti_nav_msgs/msg/route.hpp>
#include <swri_route_util/route_point.h>

namespace swri_route_util
{
// The Route class provides a more friendly interface for working with
// the marti_nav_msgs::Route message.  It provides native access to
// known properties and supports fast lookups of points by their ids.
//
// This class is optimized for a combination of simplicity, nice
// syntax, and performance for standard use cases.  This means that
// some tradeoffs were made for less common use cases.
//
// In particular, findPointId() may take a long time when called with
// ids that do not exist in the route because of the behind-the-scenes
// caching.  If this is an important use case and you are ABSOLUTELY
// sure that the points list has not been modified since the class was
// created from a message, you can avoid this performance hit by
// calling findPointIdConst() instead.
class Route
{
 public:
  // Create a new empty route.
  Route() = default;

  // Create a route from an existing marti_nav_msgs::Route message.
  // Warning: If a route or a specific route point contains properties
  // with duplicate keys, only one will be kept; the others are
  // silently discarded.  Don't use non-unique property keys.
  explicit Route(const marti_nav_msgs::msg::Route &msg);

  // Create a marti_nav_msgs::Route from the route, in place version.
  void toMsg(marti_nav_msgs::msg::Route &msg) const;
  // Create a marti_nav_msgs::Route from the route, returned as a
  // shared pointer.
  marti_nav_msgs::msg::Route::SharedPtr toMsgPtr() const;

  // The header of the route. This is exposed as field to be
  // consistent with typical ROS style.
  std_msgs::msg::Header header;

  // The points of the route.  This is exposed as a field because it's
  // much simpler (for me to write and you to use) than trying to
  // replicate the entire vector interface.
  std::vector<RoutePoint> points;

  // A route is considered invalid if it has no points.
  bool valid() const;
  
  // Find a point index by its ID.  For the common use case of looking
  // up valid ids in a static route, this will be very fast.  For less
  // common cases like looking up valid ids in a route that has been
  // modified, it may be slower.  It is the slowest when looking up
  // many invalid ids in any route.  If there are multiple points with
  // the requested key, it will return one of the points, but which
  // one is undefined.  Don't use non-unqiue ids.
  bool findPointId(size_t &index, const std::string &id) const;

  // Find a point index by its ID, the less safe version.  This
  // version should only be used on Routes with a valid internal index
  // (rebuildPointIndex() was called and points were never added,
  // removed, reordered, or changed ids).  This version will not
  // rebuild the internal point index if an id is not found, so it is
  // much faster for repeated invalid lookups.  If the "static"
  // assumption is violated, this version may incorrectly miss a valid
  // id.  It will never return a point with a different id than
  // requested.  Be sure you know what you're doing if you want to use
  // this.
  bool findPointIdConst(size_t &index, const std::string &id) const;

  // Rebuilds the internal point index.  This is handled internally
  // and exposed for very special use cases, which you probably do not
  // need.  Be sure you know what you're doing if you want to use this.
  void rebuildPointIndex() const;

  // Native access to the route "name" property, which is required to
  // exist.
  std::string name() const;
  void setName(const std::string &name);

  // Native access to the route "guid" property, which is required to
  // exist.
  std::string guid() const;
  void setGuid(const std::string &guid);

  // The following methods provide general purpose access to route
  // properties.  They will also correctly map to properties that are
  // exposed natively.  Native methods (e.g.  name(), guid()) are
  // generally faster and safer and should be preferred when
  // available.

  // Get a list of all the properties defined for this route,
  // including native properties.
  std::vector<std::string> getPropertyNames() const;

  // Get the value of a property.  Returns an empty string if the
  // property does not exist.
  std::string getProperty(const std::string &name) const;
  template <typename T>

  // Get the value of a property, lexically cast to a known type.
  // Returns the lexical cast of an empty string if the property does
  // not exist..
  T getTypedProperty(const std::string &name) const;

  // Determine if the specified property is defined for the route.
  bool hasProperty(const std::string &name) const;

  // Set the value of a property.  If the property doesn't exist, it
  // is added.
  void setProperty(const std::string &name, const std::string &value);

  // Delete a property.  If the property doesn't exist or is not
  // deletable (e.g. name, guid), this method does nothing.
  void deleteProperty(const std::string &name);

 public:
  // The point index maps point ids to their index in the points
  // vector.  It is mutable because we want to support fast look ups
  // on const Routes.
  mutable std::map<std::string, size_t> point_index_;

  // Map containing generic properties.
  std::map<std::string, std::string> properties_;

  // Storage for native properties.
  std::string guid_;
  std::string name_;
};  // class Route

// Typedef shared pointers to make migrating from the message types to
// this interface more convenient.
typedef std::shared_ptr<Route> RoutePtr;
typedef std::shared_ptr<Route const> RouteConstPtr;

template<typename T>
inline
T Route::getTypedProperty(const std::string &name) const
{
  return boost::lexical_cast<T>(getProperty(name));
}
} // namespace swri_route_util

// #include "route_serializer.h"

#endif  // SWRI_ROUTE_UTIL_ROUTE_H_
