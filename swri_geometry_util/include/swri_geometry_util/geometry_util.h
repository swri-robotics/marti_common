// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#ifndef GEOMETRY_UTIL_GEOMETRY_UTIL_H_
#define GEOMETRY_UTIL_GEOMETRY_UTIL_H_

#include <tf/transform_datatypes.h>

namespace swri_geometry_util
{
  /**
   * Calculate the distance from a point to a plane.
   *
   * @param[in]  plane_normal  The normal vector of the plane.
   * @param[in]  plane_point   A point on the plane.
   * @param[in]  point         The point to measure the distance of.
   *
   * @returns The distance of the point from the plane.
   */
  double DistanceFromPlane(
      const tf::Vector3& plane_normal,
      const tf::Vector3& plane_point,
      const tf::Vector3& point);
      
  double DistanceFromLineSegment(
      const tf::Vector3& line_start,
      const tf::Vector3& line_end,
      const tf::Vector3& point);
      
  tf::Vector3 ProjectToLineSegment(
      const tf::Vector3& line_start,
      const tf::Vector3& line_end,
      const tf::Vector3& point);

  /**
   * Find closest point to two 3D lines.
   *
   * @param[in]   a1     First point on line 1.
   * @param[in]   a2     Second point on line 1.
   * @param[in]   b1     First point on line 2.
   * @param[in]   b2     Second point on line 2.
   * @param[out]  point  The closest point to both lines.
   *
   * @returns True unless a1 == a2, b1 == b2, or lines are parallel.
   */
  bool ClosestPointToLines(
      const tf::Vector3& a1,
      const tf::Vector3& a2,
      const tf::Vector3& b1,
      const tf::Vector3& b2,
      tf::Vector3& point);
}

#endif  // GEOMETRY_UTIL_GEOMETRY_UTIL_H_
