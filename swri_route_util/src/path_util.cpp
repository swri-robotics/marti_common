// *****************************************************************************
//
// Copyright (c) 2020, Southwest Research Institute® (SwRI®)
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
#include <swri_route_util/path_util.h>

#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_util.h>

#include <swri_geometry_util/geometry_util.h>

namespace mnm = marti_nav_msgs;
namespace stu = swri_transform_util;

namespace swri_route_util
{

void transform(marti_nav_msgs::Path &path,
               const stu::Transform &transform,
               const std::string &target_frame)
{
  for (auto &point : path.points) {
    tf::Vector3 position(point.x, point.y, 0.0);
    position = transform*position;

    point.x = position.x();
    point.y = position.y();

    tf::Quaternion q = tf::createQuaternionFromYaw(point.yaw);
    point.yaw = tf::getYaw(transform*q);
  }
  path.header.frame_id = target_frame;
}

void fillOrientations(marti_nav_msgs::Path &path)
{
  // We can't estimate any orientations for 0 or 1 points.
  if (path.points.size() < 2) {
    return;
  }

  double yaw;
  for (size_t i = 0; i + 1 < path.points.size(); i++)
  {
    marti_nav_msgs::PathPoint& pt = path.points[i];
    const marti_nav_msgs::PathPoint& next_pt = path.points[i+1];

    // Calculate yaw based on the direction between these points
    yaw = atan2(
        next_pt.y - pt.y,
        next_pt.x - pt.x);

    if (path.in_reverse)
    {
      yaw += M_PI;
    }

    pt.yaw = yaw;
  }

  // fill in the last yaw
  path.points.back().yaw = path.points[path.points.size() - 2].yaw;
}


bool findLocalNearestDistanceForward(
    const marti_nav_msgs::Path& path,
    const double x, const double y,
    PathPosition& nearest_position,
    double& nearest_separation)
{
  // Handle odd sizes
  size_t num_points = path.points.size();
  if (num_points == 0)
  {
    return false;
  }
  else if (num_points == 1)
  {
    nearest_position.index = 0;
    nearest_position.distance = 0.0;
    nearest_separation = std::sqrt(std::pow(path.points[0].x - x, 2.0) +
                                   std::pow(path.points[0].y - y, 2.0));
    return true;
  }

  cv::Vec2d point(x, y);

  // Find the nearest line segment from the beginning of the path
  double min_separation_sqr = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < num_points; i++)
  {
    auto& start = path.points[i];
    auto& end = path.points[i+1];

    cv::Vec2d proj = swri_geometry_util::ProjectToLineSegment(cv::Vec2d(start.x, start.y),
                                                              cv::Vec2d(end.x, end.y),
                                                              point);

    double separation_sqr = std::pow(x - proj[0], 2.0) + std::pow(y - proj[1], 2.0);
    if (separation_sqr <= min_separation_sqr)
    {
      min_separation_sqr = separation_sqr;
      nearest_position.index = i;
      nearest_position.distance = std::sqrt(std::pow(start.x - proj[0], 2.0) +
                                            std::pow(start.y - proj[1], 2.0));
    }

    //if separation is increasing, local minima was found. stop the search
    if (separation_sqr > 2.0*min_separation_sqr)
    {
      break;
    }
  }

  nearest_separation = std::sqrt(min_separation_sqr);

  return true;
}

static void normalizePathPosition(const marti_nav_msgs::Path& path,
  PathPosition& position)
{
  // handle being past the end of the route
  if (position.index > path.points.size() - 1)
  {
    position.distance = 0.0;
    position.index = path.points.size() - 1;
    return;
  }

  double distance = position.distance;
  size_t index = position.index;
  while (distance < 0.0)
  {
    if (index == 0)
    {
      distance = 0.0;// cant be before start
      break;
    }

    auto& s = path.points[index];
    auto& e = path.points[index-1];
    double len = std::sqrt(std::pow(s.x - e.x, 2.0) +
                           std::pow(s.y - e.y, 2.0));
    if (-distance >= len)
    {
      distance += len;
      index--;
    }
    else
    {
      break;
    }
  }

  while (distance > 0.0)
  {
    if (index+1 == path.points.size())
    {
      break;
    }

    auto& s = path.points[index];
    auto& e = path.points[index+1];
    double len = std::sqrt(std::pow(s.x - e.x, 2.0) +
                           std::pow(s.y - e.y, 2.0));
    if (distance >= len)
    {
      distance -= len;
      index++;
    }
    else
    {
      break;
    }
  }

  position.index = index;
  position.distance = distance;
}

static double interpolateAngle(double from, double to, double t)
{
  from = std::fmod(from + M_PI*2.0, M_PI*2.0);
  to = std::fmod(to + M_PI*2.0, M_PI*2.0);
  double diff = std::abs(from - to);
  if (diff < M_PI)
  {
    return from*(1.0-t) + to*t;
  }
  else if (from > to)
  {
    from -= M_PI*2.0;
    return from*(1.0-t) + to*t;
  }
  else
  {
    to -= M_PI*2.0;
    return from*(1.0-t) + to*t;
  }
}

void getPathPosition(const marti_nav_msgs::Path& path,
                     const PathPosition position,
                     tf::Vector3& pos)
{
  PathPosition npos = position;
  normalizePathPosition(path, npos);

  if (npos.distance == 0.0 || npos.index == path.points.size() - 1)
  {
    auto& point = path.points[npos.index];
    pos = tf::Vector3(point.x, point.y, 0.0);
    return;
  }

  auto& start = path.points[npos.index];
  auto& end = path.points[npos.index+1];

  double distance = std::sqrt(std::pow(start.x - end.x, 2.0) + std::pow(start.y - end.y, 2.0));
  double frac = npos.distance/distance;

  pos = tf::Vector3(start.x*(1.0 - frac) + end.x*frac,
                  start.y*(1.0 - frac) + end.y*frac,
                  0.0);
}

void getPathPose(const marti_nav_msgs::Path& path,
                 const PathPosition position,
                 tf::Transform& tf,
                 const bool allow_extrapolation)
{
  PathPosition npos = position;
  normalizePathPosition(path, npos);

  if (npos.distance == 0.0 || npos.index == path.points.size() - 1)
  {
    if (npos.index == path.points.size() - 1 && npos.distance > 0.0
        && path.points.size() > 1 
        && allow_extrapolation)
    {
      // extrapolate
      auto& start = path.points[npos.index];
      auto& prev = path.points[npos.index-1];

      tf::Vector3 pos(start.x, start.y, 0.0);
      tf::Vector3 dir = tf::Vector3(start.x-prev.x, start.y-prev.y, 0.0).normalized();
      pos += npos.distance*dir;
      tf = tf::Transform(tf::createQuaternionFromYaw(start.yaw), pos);
      return;
    }

    auto& point = path.points[npos.index];
    tf = tf::Transform(tf::createQuaternionFromYaw(point.yaw),
                         tf::Vector3(point.x, point.y, 0.0));
    return;
  }

  auto& start = path.points[npos.index];
  auto& end = path.points[npos.index+1];

  double distance = std::sqrt(std::pow(start.x - end.x, 2.0) + std::pow(start.y - end.y, 2.0));
  double frac = npos.distance/distance;

  tf::Vector3 pos(start.x*(1.0 - frac) + end.x*frac,
                  start.y*(1.0 - frac) + end.y*frac,
                  0.0);
  double yaw = interpolateAngle(start.yaw, end.yaw, frac);
  tf = tf::Transform(tf::createQuaternionFromYaw(yaw), pos);
}

}  // namespace swri_route_util
