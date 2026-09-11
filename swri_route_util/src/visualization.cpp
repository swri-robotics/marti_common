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
#include <chrono>
#include <string>

#include <swri_route_util/visualization.h>
#include <swri_route_util/util.h>

namespace swri_route_util
{
static geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = 0.0;
  return pt;
}

void markerForRouteSpeeds(
  visualization_msgs::msg::Marker &m,
  const Route &route,
  const marti_nav_msgs::msg::RouteSpeedArray &speeds,
  double scale)
{
  m.header.frame_id = route.header.frame_id;
  m.header.stamp = rclcpp::Clock().now();
  // m.ns = ;
  // m.id = ;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = rclcpp::Duration(std::chrono::nanoseconds::zero());
  m.frame_locked = false;

  m.points.reserve(speeds.speeds.size()*2);

  for (auto const &speed : speeds.speeds) {
    marti_nav_msgs::msg::RoutePosition position;
    position.id = speed.id;
    position.distance = speed.distance;

    RoutePoint p;
    if (!interpolateRoutePosition(p, route, position, true)) {
      continue;
    }

    tf2::Vector3 p1 = p.position();
    tf2::Vector3 v = tf2::Transform(p.orientation()) * tf2::Vector3(0.0, 1.0, 0.0);
    tf2::Vector3 p2 = p1 + scale*speed.speed*v;

    m.points.push_back(makePoint(p1.x(), p1.y()));
    m.points.push_back(makePoint(p2.x(), p2.y()));
  }
}

void markerArrayForObstacles(
  visualization_msgs::msg::MarkerArray &markers,
  const marti_nav_msgs::msg::ObstacleArray &obstacles,
  const std::string &ns,
  const std_msgs::msg::ColorRGBA &color,
  double line_width)
{
  markers.markers.clear();
  markers.markers.reserve(obstacles.obstacles.size());

  for (auto const &obstacle : obstacles.obstacles) {
    // A line strip needs at least two points to draw anything.
    if (obstacle.polygon.size() < 2) {
      continue;
    }

    visualization_msgs::msg::Marker m;
    m.header = obstacles.header;
    m.ns = ns;
    m.id = static_cast<int>(markers.markers.size());
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    // The obstacle's polygon is defined relative to the obstacle's
    // pose, so the marker is placed at that pose and the polygon
    // points are used without modification.
    m.pose = obstacle.pose;
    // Messages default to the identity rotation, but a publisher that
    // builds the pose from raw data can leave the quaternion zeroed
    // out, which is not a valid rotation and can not be rendered.
    if (m.pose.orientation.x == 0.0 &&
        m.pose.orientation.y == 0.0 &&
        m.pose.orientation.z == 0.0 &&
        m.pose.orientation.w == 0.0) {
      m.pose.orientation.w = 1.0;
    }
    m.scale.x = line_width;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color = color;
    m.lifetime = rclcpp::Duration(std::chrono::nanoseconds::zero());
    m.frame_locked = false;

    m.points = obstacle.polygon;
    // The polygon is implicitly closed by a segment between the last
    // and first points.
    m.points.push_back(obstacle.polygon.front());

    markers.markers.push_back(m);
  }
}
}  // namespace swri_route_util
