// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <swri_route_util/visualization.h>

namespace
{
geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = 0.0;
  return pt;
}

marti_nav_msgs::msg::Obstacle makeObstacle(
  const std::string &id,
  const std::vector<geometry_msgs::msg::Point> &polygon)
{
  marti_nav_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.polygon = polygon;
  return obstacle;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

const std::vector<geometry_msgs::msg::Point> kSquare = {
  makePoint(0.0, 0.0),
  makePoint(2.0, 0.0),
  makePoint(2.0, 2.0),
  makePoint(0.0, 2.0)};
}  // namespace

TEST(MarkerArrayForObstacles, DrawsAClosedLineStripPerObstacle)
{
  marti_nav_msgs::msg::ObstacleArray obstacles;
  obstacles.header.frame_id = "map";
  obstacles.header.stamp.sec = 42;
  obstacles.obstacles.push_back(makeObstacle("square", kSquare));
  obstacles.obstacles.back().pose.position.x = 10.0;
  obstacles.obstacles.back().pose.position.y = 20.0;
  obstacles.obstacles.back().pose.orientation.w = 1.0;

  visualization_msgs::msg::MarkerArray markers;
  swri_route_util::markerArrayForObstacles(
    markers, obstacles, "obstacles", makeColor(1.0, 0.0, 0.0, 1.0), 0.25);

  ASSERT_EQ(1U, markers.markers.size());
  const visualization_msgs::msg::Marker &m = markers.markers.front();

  EXPECT_EQ("map", m.header.frame_id);
  EXPECT_EQ(42, m.header.stamp.sec);
  EXPECT_EQ("obstacles", m.ns);
  EXPECT_EQ(0, m.id);
  EXPECT_EQ(visualization_msgs::msg::Marker::LINE_STRIP, m.type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, m.action);
  EXPECT_DOUBLE_EQ(0.25, m.scale.x);
  EXPECT_FLOAT_EQ(1.0, m.color.r);
  EXPECT_FLOAT_EQ(1.0, m.color.a);

  // The marker is placed at the obstacle's pose so that the polygon
  // points can be used without modification.
  EXPECT_DOUBLE_EQ(10.0, m.pose.position.x);
  EXPECT_DOUBLE_EQ(20.0, m.pose.position.y);

  // The polygon is implicitly closed, so the first point is repeated.
  ASSERT_EQ(kSquare.size() + 1, m.points.size());
  for (size_t i = 0; i < kSquare.size(); i++) {
    EXPECT_DOUBLE_EQ(kSquare[i].x, m.points[i].x) << "point " << i;
    EXPECT_DOUBLE_EQ(kSquare[i].y, m.points[i].y) << "point " << i;
  }
  EXPECT_DOUBLE_EQ(kSquare.front().x, m.points.back().x);
  EXPECT_DOUBLE_EQ(kSquare.front().y, m.points.back().y);
}

TEST(MarkerArrayForObstacles, SkipsObstaclesThatCanNotBeDrawn)
{
  marti_nav_msgs::msg::ObstacleArray obstacles;
  obstacles.obstacles.push_back(makeObstacle("empty", {}));
  obstacles.obstacles.push_back(makeObstacle("single_point", {makePoint(1.0, 1.0)}));
  obstacles.obstacles.push_back(makeObstacle("square", kSquare));

  visualization_msgs::msg::MarkerArray markers;
  swri_route_util::markerArrayForObstacles(
    markers, obstacles, "obstacles", makeColor(0.0, 1.0, 0.0, 1.0), 0.1);

  // Only the square can be drawn, and the ids stay sequential so that
  // stale markers can be deleted by id.
  ASSERT_EQ(1U, markers.markers.size());
  EXPECT_EQ(0, markers.markers.front().id);
  EXPECT_EQ(kSquare.size() + 1, markers.markers.front().points.size());
}

TEST(MarkerArrayForObstacles, NumbersMarkersSequentially)
{
  marti_nav_msgs::msg::ObstacleArray obstacles;
  for (int i = 0; i < 3; i++) {
    obstacles.obstacles.push_back(makeObstacle("obstacle", kSquare));
  }

  visualization_msgs::msg::MarkerArray markers;
  swri_route_util::markerArrayForObstacles(
    markers, obstacles, "ns", makeColor(0.0, 0.0, 1.0, 1.0), 0.1);

  ASSERT_EQ(3U, markers.markers.size());
  for (size_t i = 0; i < markers.markers.size(); i++) {
    EXPECT_EQ(static_cast<int>(i), markers.markers[i].id);
    EXPECT_EQ("ns", markers.markers[i].ns);
  }
}

TEST(MarkerArrayForObstacles, RepairsAnUnsetOrientation)
{
  marti_nav_msgs::msg::ObstacleArray obstacles;
  obstacles.obstacles.push_back(makeObstacle("unset", kSquare));
  // A quaternion that has been zeroed out is not a valid rotation and
  // can not be rendered.  Messages default to the identity rotation,
  // so this has to be done explicitly.
  obstacles.obstacles.front().pose.orientation.x = 0.0;
  obstacles.obstacles.front().pose.orientation.y = 0.0;
  obstacles.obstacles.front().pose.orientation.z = 0.0;
  obstacles.obstacles.front().pose.orientation.w = 0.0;

  obstacles.obstacles.push_back(makeObstacle("rotated", kSquare));
  obstacles.obstacles.back().pose.orientation.z = 0.5;
  obstacles.obstacles.back().pose.orientation.w = 0.5;

  visualization_msgs::msg::MarkerArray markers;
  swri_route_util::markerArrayForObstacles(
    markers, obstacles, "obstacles", makeColor(1.0, 1.0, 1.0, 1.0), 0.1);

  ASSERT_EQ(2U, markers.markers.size());
  EXPECT_DOUBLE_EQ(1.0, markers.markers[0].pose.orientation.w);

  // An orientation that was actually set is left alone.
  EXPECT_DOUBLE_EQ(0.5, markers.markers[1].pose.orientation.z);
  EXPECT_DOUBLE_EQ(0.5, markers.markers[1].pose.orientation.w);
}

TEST(MarkerArrayForObstacles, ClearsAnyExistingMarkers)
{
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.resize(5);

  marti_nav_msgs::msg::ObstacleArray obstacles;
  obstacles.obstacles.push_back(makeObstacle("square", kSquare));

  swri_route_util::markerArrayForObstacles(
    markers, obstacles, "obstacles", makeColor(1.0, 0.0, 0.0, 1.0), 0.1);

  EXPECT_EQ(1U, markers.markers.size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
