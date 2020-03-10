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
#ifndef SWRI_ROUTE_UTIL_UTIL_H_
#define SWRI_ROUTE_UTIL_UTIL_H_

#include <marti_nav_msgs/msg/route_position.hpp>
#include <swri_transform_util/transform.h>

namespace swri_route_util
{
class Route;
class RoutePoint;

// Transform a route.  The route will be transformed in place using
// the supplied transform.  Known property types that are affected by
// the transform are expected to be transformed properly.  The
// frame_id of the transformed route will be set to the required
// target_frame argument, because forgetting to up the frame_id has
// caused difficult bugs several times.
void transform(Route &route,
               const swri_transform_util::Transform &transform,
               const std::string &target_frame);


// Project a route to the XY plane by setting the Z coordinate to zero.
void projectToXY(Route &route);


// Fill in the orientation of the route points using an estimate from
// the route geometry and desired "up" direction.  This function
// assumes the route is in a cartesian (e.g. not WGS84) frame.
void fillOrientations(Route &route,
                      const tf2::Vector3 &up=tf2::Vector3(0.0, 0.0, 1.0),
                      rclcpp::Logger logger = rclcpp::get_logger("swri_transform_util::fillOrientations"));


// Find the closest point on the route (as a route position) for a
// given point.  If extrapolate_before_start and/or
// extrapolate_past_end are true, the projection will consider the
// first and last segments to extend infinitely (ONLY if the point is
// nearest to either without extrapolation).  This function assumes
// the route is in a cartesian (e.g. not WGS84) frame.
bool projectOntoRoute(marti_nav_msgs::msg::RoutePosition &position,
                      const Route &route,
                      const tf2::Vector3 &point,
                      bool extrapolate_before_start,
                      bool extrapolate_past_end);


// Find the closest position on a route for a given point, restricted
// to a subset of the route.  The subset is defined by a start and end
// position on the route.  This function assumes the route is in a
// cartesian (e.g. not WGS84) frame.
bool projectOntoRouteWindow(marti_nav_msgs::msg::RoutePosition &position,
                            const Route &route,
                            const tf2::Vector3 &point,
                            const marti_nav_msgs::msg::RoutePosition &window_start,
                            const marti_nav_msgs::msg::RoutePosition &window_end);


// Normalize a route position.  A normalize route position is guaranteed to
// have:
//   - A valid id for a point in the route.
//   
//   - If the position is before the start of the route, the id will be
//     first route point and the distance will be negative.

//   - If the position is after the end of the route, the id will be the
//     last route point and the distance will be positive.
//
//   - Otherwise, the position's id will be for the point that begins
//     the segment containing the position, and the distance will be
//     less than that length of that segment.
//
//  This function fails if the original position's id is not found in the
// route.  This function assumes the route is in a cartesian (e.g. not
// WGS84) frame.
bool normalizeRoutePosition(marti_nav_msgs::msg::RoutePosition &normalized_position,
                            const Route &route,
                            const marti_nav_msgs::msg::RoutePosition &position);


// Create a route point from a route position by interpolating between
// the route's points as needed.  This function assumes the route is
// in a cartesian (e.g. not WGS84) frame.
bool interpolateRoutePosition(RoutePoint &point,
                              const Route &route,
                              const marti_nav_msgs::msg::RoutePosition &position,
                              bool allow_extrapolation);

// Return the distance between two route positions.  This function
// works for routes defined in WGS84 or Euclidean spaces.
bool routeDistance(
  double &distance,
  const marti_nav_msgs::msg::RoutePosition &start,
  const marti_nav_msgs::msg::RoutePosition &end,
  const Route &route);

// Return the distances between a start route position and multiple
// end route positions.  This function works for routes defined in
// WGS84 or Euclidean spaces.  This function returns false if the
// start point wasn't found in the route, in which case distances is
// not modified.  If the function is true, the start point was found
// and distances is guaranteed to be the same size as ends.  If an end
// point is not found in the route, its distance is set to NaN.
bool routeDistances(
  std::vector<double> &distances,
  const marti_nav_msgs::msg::RoutePosition &start,
  const std::vector<marti_nav_msgs::msg::RoutePosition> &ends,
  const Route &route);

// Extracts a subroute from [start, end)
bool extractSubroute(
  Route &sub_route,
  const Route &route,
  const marti_nav_msgs::msg::RoutePosition &start,
  const marti_nav_msgs::msg::RoutePosition &end);
}  // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_UTIL_H_
