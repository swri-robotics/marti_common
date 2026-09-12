// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROUTE_UTIL_VISUALIZATION_H_
#define SWRI_ROUTE_UTIL_VISUALIZATION_H_

#include <string>

#include <swri_route_util/route.h>
#include <marti_nav_msgs/msg/obstacle_array.hpp>
#include <marti_nav_msgs/msg/route_speed_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace swri_route_util
{
// Create a marker for a set of speeds along a route.  Each speed will
// be drawn as a line that starts at the speed's location and extends
// perpendicular to the route.  The length of the line corresponds to
// the speed.  The scale parameter sets the length per m/s scale.  The
// marker's namespace and id will still need to be filled out. The
// line colors defaults to black but can be changed after the marker
// has been built.
void markerForRouteSpeeds(
  visualization_msgs::msg::Marker &marker,
  const Route &route,
    const marti_nav_msgs::msg::RouteSpeedArray &speeds,
    double scale);

// Create markers that outline a set of obstacles.  Each obstacle is
// drawn as a closed line strip, using the obstacle's pose as the
// marker's pose so that the obstacle's polygon can be used as-is.
// The markers are all created in the namespace 'ns' and are numbered
// sequentially starting at zero.  Obstacles that do not have enough
// points to draw a line are skipped, so the number of markers is not
// necessarily the number of obstacles.  Any markers already in the
// array are removed.
void markerArrayForObstacles(
  visualization_msgs::msg::MarkerArray &markers,
  const marti_nav_msgs::msg::ObstacleArray &obstacles,
  const std::string &ns,
  const std_msgs::msg::ColorRGBA &color,
  double line_width);
}  // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_VISUALIZATION_H_
