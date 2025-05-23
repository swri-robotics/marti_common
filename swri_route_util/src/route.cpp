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
#include <swri_route_util/route.h>
#include <swri_route_util/route_point.h>

namespace mnm = marti_nav_msgs;

namespace swri_route_util
{
// Helper function used by the Route(mnm::Route) constructor.
  static
  void pointFromMsg(RoutePoint& dst, const marti_nav_msgs::msg::RoutePoint& src)
  {
    dst.setPose(src.pose);
    dst.setId(src.id);

    for (auto const& prop : src.properties)
    {
      dst.setProperty(prop.key, prop.value);
    }
  }

  Route::Route(const mnm::msg::Route& msg)
  {
    header = msg.header;

    points.resize(msg.route_points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
      pointFromMsg(points[i], msg.route_points[i]);
    }

    for (auto const& prop : msg.properties)
    {
      setProperty(prop.key, prop.value);
    }

    rebuildPointIndex();
  }

// Helper method used by the toMsg() method.
  static
  void msgFromPoint(marti_nav_msgs::msg::RoutePoint& dst, const RoutePoint& src)
  {
    dst.pose = src.poseMsg();
    dst.id = src.id();

    std::vector<std::string> names = src.getPropertyNames();
    dst.properties.resize(names.size());
    for (size_t i = 0; i < names.size(); ++i)
    {
      dst.properties[i].key = names[i];
      dst.properties[i].value = src.getProperty(names[i]);
    }
  }

  void Route::toMsg(mnm::msg::Route& msg) const
  {
    msg.header = header;

    msg.route_points.resize(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
      msgFromPoint(msg.route_points[i], points[i]);
    }

    std::vector<std::string> names = getPropertyNames();
    msg.properties.resize(names.size());
    for (size_t i = 0; i < names.size(); ++i)
    {
      msg.properties[i].key = names[i];
      msg.properties[i].value = getProperty(names[i]);
    }
  }

  mnm::msg::Route::SharedPtr Route::toMsgPtr() const
  {
    mnm::msg::Route::SharedPtr ptr = std::make_shared<mnm::msg::Route>();
    toMsg(*ptr);
    return ptr;
  }

  bool Route::valid() const
  {
    return !points.empty();
  }

  bool Route::findPointId(size_t& index, const std::string& id) const
  {
    if (point_index_.count(id))
    {
      size_t i = point_index_.at(id);
      if (i < points.size() && points[i].id() == id)
      {
        // This is a cache hit!
        index = i;
        return true;
      }

      // This is cache miss... our cache is out of date.
    }

    // If we reach here, either our cache is out of date or the point
    // doesn't exist in the cache.  We will rebuild the cache index to
    // make sure it is current.
    rebuildPointIndex();

    if (point_index_.count(id))
    {
      // If the point was found, we expect the id to be valid (unless
      // you're using this class from multiple threads, which is not
      // supported.
      index = point_index_.at(id);
      return true;
    }

    // The point does not exist in this route.
    return false;
  }

  bool Route::findPointIdConst(size_t& index, const std::string& id) const
  {
    if (point_index_.count(id))
    {
      size_t i = point_index_.at(id);
      if (i < points.size() && points[i].id() == id)
      {
        // This is a cache hit!
        index = i;
        return true;
      }
      // This is cache miss... our cache is out of date.
    }

    return false;
  }

  std::vector<std::string> Route::getPropertyNames() const
  {
    std::vector<std::string> names;
    names.emplace_back("name");
    names.emplace_back("guid");

    for (auto const& it : properties_)
    {
      names.push_back(it.first);
    }

    return names;
  }

  std::string Route::getProperty(const std::string& name) const
  {
    if (name == "name")
    {
      return name_;
    }
    else if (name == "guid")
    {
      return guid_;
    }
    else if (properties_.count(name))
    {
      return properties_.at(name);
    }
    else
    {
      return "";
    }
  }

  bool Route::hasProperty(const std::string& name) const
  {
    if (name == "name")
    {
      return true;
    }
    else if (name == "guid")
    {
      return true;
    }
    else
    {
      return properties_.count(name) > 0;
    }
  }

  void Route::setProperty(const std::string& name, const std::string& value)
  {
    if (name == "name")
    {
      name_ = value;
    }
    else if (name == "guid")
    {
      guid_ = value;
    }
    else
    {
      properties_[name] = value;
    }
  }

  void Route::deleteProperty(const std::string& name)
  {
    // If we add "native" properties that are erasable, we should check
    // for those here first and mark them as deleted when appropriate.

    // Otherwise, fall back to the generic properties.
    // std::map::erase() ignores the call if the key is not found.
    properties_.erase(name);
  }

  std::string Route::name() const
  {
    return name_;
  }

  void Route::setName(const std::string& name)
  {
    name_ = name;
  }

  std::string Route::guid() const
  {
    return guid_;
  }

  void Route::setGuid(const std::string& guid)
  {
    guid_ = guid;
  }

  void Route::rebuildPointIndex() const
  {
    // Throw away the old index.
    point_index_.clear();
    for (size_t i = 0; i < points.size(); ++i)
    {
      point_index_[points[i].id()] = i;
    }
  }
}  // namespace swri_route_util
