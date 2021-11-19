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
#ifndef SWRI_ROUTE_UTIL_PATH_UTIL_H_
#define SWRI_ROUTE_UTIL_PATH_UTIL_H_

#include <marti_nav_msgs/Plan.h>
#include <marti_nav_msgs/PlanPosition.h>
#include <swri_transform_util/transform.h>

namespace swri_route_util
{
// Transform a plan.  The path will be transformed in place using
// the supplied transform. The
// frame_id of the transformed route will be set to the required
// target_frame argument, because forgetting to up the frame_id has
// caused difficult bugs several times.
void transform(marti_nav_msgs::Plan &path,
               const swri_transform_util::Transform &transform,
               const std::string &target_frame);

// Fill in the orientation of the path points using an estimate from
// the path geometry.  This function
// assumes the route is in a cartesian (e.g. not WGS84) frame.
void fillOrientations(marti_nav_msgs::Plan &path);

// Set z's to zeros
void projectToXY(marti_nav_msgs::Plan &route);


// Finds the nearest point on a path to a given position starting
// from the first point in the path
bool findLocalNearestDistanceForward(
    const marti_nav_msgs::Plan& path,
    const double x, const double y,
    marti_nav_msgs::PlanPosition& nearest_position,
    double& nearest_separation);

// Normalizes a position along a plan so that it refers to the closest point
void normalizePlanPosition(marti_nav_msgs::PlanPosition& position,
  const marti_nav_msgs::Plan& path);

// Gets a transform representing a position along a route
// Automatically normalizes the path positions
void getPathPose(const marti_nav_msgs::Plan& path,
                 const marti_nav_msgs::PlanPosition position,
                 tf::Transform& tf,
                 const bool allow_extrapolation = false);

// Gets a vector3 representing a position along a route
// Automatically normalizes the path positions
void getPlanPosition(const marti_nav_msgs::Plan& path,
                 const marti_nav_msgs::PlanPosition position,
                 tf::Vector3& tf,
                 bool extrapolate = false);

// Gets an interpolated planpoint at the given position
void interpolatePlanPosition(const marti_nav_msgs::Plan& path,
                 const marti_nav_msgs::PlanPosition position,
                 marti_nav_msgs::PlanPoint& pt,
                 bool extrapolate = false);

// Small utility function to make a tf::Vector3 from a PlanPoint
inline tf::Vector3 getPointPosition(const marti_nav_msgs::PlanPoint& pt)
{
  return tf::Vector3(pt.x, pt.y, pt.z);
}

// Gets the nearest PlanPosition for a given point along a plan
bool projectOntoPlan(marti_nav_msgs::PlanPosition &position,
                      const marti_nav_msgs::Plan &route,
                      const tf::Vector3 &point,
                      bool extrapolate_before_start,
                      bool extrapolate_past_end);

// Gets the nearest PlanPosition for a given point along a subsection of a plan
bool projectOntoPlanWindow(
  marti_nav_msgs::PlanPosition &position,
  const marti_nav_msgs::Plan &route,
  const tf::Vector3 &point,
  const marti_nav_msgs::PlanPosition &window_start,
  const marti_nav_msgs::PlanPosition &window_end);

// Get signed distance between two positions along a plan
// Handles both wgs84 and cartesian frames
bool planDistance(
  double &distance,
  const marti_nav_msgs::PlanPosition &start,
  const marti_nav_msgs::PlanPosition &end,
  const marti_nav_msgs::Plan &route);
}
#endif  // SWRI_ROUTE_UTIL_UTIL_H_
