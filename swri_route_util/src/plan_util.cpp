// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
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
#include <swri_route_util/plan_util.h>

#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_util.h>

#include <swri_geometry_util/geometry_util.h>

namespace mnm = marti_nav_msgs;
namespace stu = swri_transform_util;

namespace swri_route_util
{

void transform(marti_nav_msgs::Plan &path,
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

void projectToXY(marti_nav_msgs::Plan &route)
{
  for (auto &point : route.points) {
    point.z = 0.0;
    // todo(exjohnson): if orientation is valid, project it to a
    // rotation around the Z axis only.
  }
}

void fillOrientations(marti_nav_msgs::Plan &path)
{
  // We can't estimate any orientations for 0 or 1 points.
  if (path.points.size() < 2) {
    return;
  }

  double yaw;
  for (size_t i = 0; i + 1 < path.points.size(); i++)
  {
    auto& pt = path.points[i];
    const auto& next_pt = path.points[i+1];

    // Calculate yaw based on the direction between these points
    yaw = atan2(
        next_pt.y - pt.y,
        next_pt.x - pt.x);

    if (pt.flags & marti_nav_msgs::PlanPoint::FLAG_REVERSE)
    {
      yaw += M_PI;
    }

    pt.yaw = yaw;
  }

  // fill in the last yaw
  path.points.back().yaw = path.points[path.points.size() - 2].yaw;
}


bool findLocalNearestDistanceForward(
    const marti_nav_msgs::Plan& path,
    const double x, const double y,
    marti_nav_msgs::PlanPosition& nearest_position,
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

void normalizePlanPosition(marti_nav_msgs::PlanPosition& position,
  const marti_nav_msgs::Plan& path)
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

// This is a private helper function that finds the nearest distance
// between a point p and the line segment defined by p0 ad p1.  It
// also returns the distance along the segment to the point that is
// nearest to p.  If extrapolate_start/extrapolate_end are true, the
// calculation considers the line segment to extend infinitely in the
// corresponding directions.
static
void nearestDistanceToLineSegment(
  double &min_distance_from_line,
  double &min_distance_on_line,
  const tf::Vector3 &p0,
  const tf::Vector3 &p1,
  const tf::Vector3 &p,
  bool extrapolate_start,
  bool extrapolate_end)
{
  tf::Vector3 v = p1 - p0;
  const double v_len_sq = v.dot(v);

  // s will be the normalized distance along v that is closest to the
  // desired point.
  double s = 0.0;
  if (v_len_sq > 1e-6) {
    s = v.dot(p - p0) / v_len_sq;
  } else {
    // The two points are too close to define a reasonable line
    // segment, so just pick p1 as the closest point.
    s = 1.0;
  }

  // If we don't allow extrapolation and the nearest point is beyond
  // the line segment boundaries, we need to clamp to the boundary.
  if (!extrapolate_start && s < 0.0) {
    s = 0.0;
  } else if (!extrapolate_end && s > 1.0) {
    s = 1.0;
  }

  tf::Vector3 x_nearest = p0 + s*v;

  min_distance_from_line = x_nearest.distance(p);
  min_distance_on_line = s*std::sqrt(v_len_sq);
}

bool projectOntoPlan(mnm::PlanPosition &position,
                      const mnm::Plan &route,
                      const tf::Vector3 &point,
                      bool extrapolate_before_start,
                      bool extrapolate_past_end)
{
  if (route.points.size() == 0) {
    // We can't do anything with this.
    return false;
  }

  if (route.points.size() == 1) {
    // We can't do much with this.
    position.index = 0;
    position.distance = 0.0;
    return true;
  }

  // First we find the nearest point on the route, without allowing
  // extrapolation.
  double min_distance_from_line = std::numeric_limits<double>::infinity();
  double min_distance_on_line = std::numeric_limits<double>::infinity();
  size_t min_segment_index = 0;

  for (size_t i = 0; i+1 < route.points.size(); ++i) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 getPointPosition(route.points[i+0]),
                                 getPointPosition(route.points[i+1]),
                                 point,
                                 false, false);

    if (distance_from_line <= min_distance_from_line) {
      min_segment_index = i;
      min_distance_on_line = distance_on_line;
      min_distance_from_line = distance_from_line;
    }
  }

  // If the nearest segment is the first or last segment, we redo
  // the search while allowing the segment to be extrapolated
  // backward or forward, respectively.  This allows graceful
  // operation if the vehicle is past the boundary of the route.

  if (extrapolate_before_start && min_segment_index == 0) {
    size_t i = 0;
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 getPointPosition(route.points[i+0]),
                                 getPointPosition(route.points[i+1]),
                                 point,
                                 true, false);
  } else if (min_segment_index + 2 == route.points.size()) {
    // The end of the route is a special case.  If we go past the end,
    // we want to return a position with the id of the last point and
    // the distance past it.  This annoying complicates things in a
    // number of places, but makes it easy to check if a point is past
    // the end of a route.
    size_t i = min_segment_index;
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 getPointPosition(route.points[i+0]),
                                 getPointPosition(route.points[i+1]),
                                 point,
                                 false, true);

    double last_length = (getPointPosition(route.points[i+1]) - getPointPosition(route.points[i+0])).length();
    if (min_distance_on_line > last_length) {
      min_segment_index++;
      min_distance_on_line -= last_length;
    }

    if (!extrapolate_past_end) {
      min_distance_on_line = 0.0;
    }
  }

  position.index = min_segment_index;
  position.distance = min_distance_on_line;
  return true;
}

bool projectOntoPlanWindow(
  marti_nav_msgs::PlanPosition &position,
  const marti_nav_msgs::Plan &route,
  const tf::Vector3 &point,
  const marti_nav_msgs::PlanPosition &window_start,
  const marti_nav_msgs::PlanPosition &window_end)
{
  if (route.points.size() < 2) {
    // We can't do anything with this.
    return false;
  }

  // First we normalize the window boundaries.
  auto start = window_start;
  normalizePlanPosition(start, route);
  auto end = window_end;
  normalizePlanPosition(end, route);

  // Handle the special case where the start and end points are
  // identical.
  if (start.index == end.index && start.distance == end.distance) {
    position = start;
    return true;
  }

  // Find the indices of the start and end points.  Since we have
  // normalized positions, we know they exist in the route.
  size_t start_index = start.index;
  size_t end_index = end.index;

  // Fix the ordering so that start comes before end.
  if ((end_index < start_index) ||
      (start_index == end_index && end.distance < start.distance)) {
    std::swap(end, start);
    std::swap(start_index, end_index);
  }

  // If either of the points are past the end of the route, we want to
  // back them up to the previous segment to reduce the number of
  // special cases we have to handle.
  if (start_index+1 == route.points.size()) {
    start_index -= 1;
    start.distance += (getPointPosition(route.points[start_index+1]) -
                       getPointPosition(route.points[start_index+0])).length();
  }
  if (end_index+1 == route.points.size()) {
    end_index -= 1;
    end.distance += (getPointPosition(route.points[end_index+1]) -
                     getPointPosition(route.points[end_index+0])).length();
  }

  // Although it causes a little duplication, it's easier over all to
  // explicitly handle the special case where the window is over a
  // single segment.
  if (start_index == end_index) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 getPointPosition(route.points[start_index+0]),
                                 getPointPosition(route.points[start_index+1]),
                                 point,
                                 true, true);

    if (distance_on_line < start.distance) {
      distance_on_line = start.distance;
    } else if (distance_on_line > end.distance) {
      distance_on_line = end.distance;
    }

    marti_nav_msgs::PlanPosition denormal_position;
    denormal_position.index = start.index;
    denormal_position.distance = distance_on_line;
    position = denormal_position;
    normalizePlanPosition(position, route);

    //position.route_id = route.guid();
    return true;
  }

  // Find the nearest point on the route, without allowing
  // extrapolation.
  double min_distance_from_line = std::numeric_limits<double>::infinity();
  double min_distance_on_line = std::numeric_limits<double>::infinity();
  size_t min_segment_index = 0;

  for (size_t i = start_index; i <= end_index; ++i) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 getPointPosition(route.points[i+0]),
                                 getPointPosition(route.points[i+1]),
                                 point,
                                 false, false);

    if (distance_from_line <= min_distance_from_line) {
      min_segment_index = i;
      min_distance_on_line = distance_on_line;
      min_distance_from_line = distance_from_line;
    }
  }

  // We have identified the closest segment.  We need to clamp it
  // to the window boundaries.
  if (min_segment_index == start_index) {
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 getPointPosition(route.points[min_segment_index+0]),
                                 getPointPosition(route.points[min_segment_index+1]),
                                 point,
                                 true, false);
    if (min_distance_on_line < start.distance) {
      min_distance_on_line = start.distance;
    }
  } else if (min_segment_index == end_index) {
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 getPointPosition(route.points[min_segment_index+0]),
                                 getPointPosition(route.points[min_segment_index+1]),
                                 point,
                                 false, true);
    if (min_distance_on_line > end.distance) {
      min_distance_on_line = end.distance;
    }
  }

  marti_nav_msgs::PlanPosition denormal_position;
  denormal_position.index = min_segment_index;
  denormal_position.distance = min_distance_on_line;
  position = denormal_position;
  normalizePlanPosition(position, route);

  return true;
}

bool planDistance(
  double &distance,
  const marti_nav_msgs::PlanPosition &start,
  const marti_nav_msgs::PlanPosition &end,
  const mnm::Plan &route)
{
  size_t start_index = start.index;
  size_t end_index = end.index;

  size_t min_index = std::min(start_index, end_index);
  size_t max_index = std::max(start_index, end_index);

  double d = 0.0;
  if (route.header.frame_id == stu::_wgs84_frame) {
    for (size_t i = min_index; i < max_index; i++) {
      d += stu::GreatCircleDistance(getPointPosition(route.points[i+1]), getPointPosition(route.points[i]));
    }
  } else {
    for (size_t i = min_index; i < max_index; i++) {
      d += (getPointPosition(route.points[i+1]) - getPointPosition(route.points[i])).length();
    }
  }

  if (end_index < start_index) {
    d = -d;
  }

  distance = d + end.distance - start.distance;
  return true;
}

void interpolatePlanPosition(const marti_nav_msgs::Plan& path,
                 const marti_nav_msgs::PlanPosition position,
                 marti_nav_msgs::PlanPoint& pt,
                 bool extrapolate)
{
  auto npos = position;
  normalizePlanPosition(npos, path);

  if (npos.distance == 0.0 || npos.index == path.points.size() - 1)
  {
    pt = path.points[npos.index];
    return;
  }

  auto& start = path.points[npos.index];
  auto& end = path.points[npos.index+1];

  double distance = std::sqrt(std::pow(start.x - end.x, 2.0) + std::pow(start.y - end.y, 2.0));
  double frac = npos.distance/distance;

  if (!extrapolate)
  {
    frac = std::min(1.0, frac);
    frac = std::max(0.0, frac);
  }

  pt = start;
  pt.x = start.x*(1.0 - frac) + end.x*frac;
  pt.y = start.y*(1.0 - frac) + end.y*frac;
  pt.z = start.z*(1.0 - frac) + end.z*frac;
  pt.yaw = interpolateAngle(start.yaw, end.yaw, frac);
}

void getPlanPosition(const marti_nav_msgs::Plan& path,
                     const marti_nav_msgs::PlanPosition position,
                     tf::Vector3& pos,
                     bool extrapolate)
{
  auto npos = position;
  normalizePlanPosition(npos, path);

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

  if (!extrapolate)
  {
    frac = std::min(1.0, frac);
    frac = std::max(0.0, frac);
  }

  pos = tf::Vector3(start.x*(1.0 - frac) + end.x*frac,
                    start.y*(1.0 - frac) + end.y*frac,
                    start.z*(1.0 - frac) + end.z*frac);
}

}  // namespace swri_route_util
