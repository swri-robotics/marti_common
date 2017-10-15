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
#include <swri_route_util/route_point.h>

namespace swri_route_util
{
RoutePoint::RoutePoint()
{
  id_ = "<not-initialized>";
  stop_point_ = false;
}

std::vector<std::string> RoutePoint::getPropertyNames() const
{
  std::vector<std::string> names;
  names.push_back("stop_point");
  names.push_back("stop_point_delay");

  for (auto const &it : properties_) {
    names.push_back(it.first);
  }
  
  return names;
}

std::string RoutePoint::getProperty(const std::string &name) const
{
  if (name == "stop_point") {
    return stop_point_ ? "true" : "false";
  }

  if (name == "stop_point_delay") {
    return boost::lexical_cast<std::string>(stop_point_delay_);
  }

  if (properties_.count(name)) {
    return properties_.at(name);
  } else {
    return "";
  }
}

bool RoutePoint::hasProperty(const std::string &name) const
{
  if (name == "stop_point") { return true; }
  if (name == "stop_point_delay") { return true; }
  return properties_.count(name);
}

void RoutePoint::setProperty(const std::string &name, const std::string &value)
{
  if (name == "stop_point") {
    stop_point_ = (value == "1") || (value == "true");
  } else if (name == "stop_point_delay") {
    stop_point_delay_ = boost::lexical_cast<double>(value);
  } else {
    properties_[name] = value;
  }
}

void RoutePoint::deleteProperty(const std::string &name)
{
  // If we add "native" properties that are erasable, we should check
  // for those here first and mark them as deleted when appropriate.
  
  // Otherwise, fall back to the generic properties.
  // std::map::erase() ignores the call if the key is not found.
  properties_.erase(name);
}
}  // namespace swri_route_util
