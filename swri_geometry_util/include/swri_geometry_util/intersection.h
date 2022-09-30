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

#ifndef SWRI_GEOMETRY_UTIL_INTERSECTION_H_
#define SWRI_GEOMETRY_UTIL_INTERSECTION_H_

#include <vector>
#include <opencv2/core/core.hpp>

#define HAVE_INT64_T_64  # Prevents conflict with OpenCV typedef of int64
#include <geos_c.h>
#undef HAVE_INT64_T_64

namespace swri_geometry_util
{
  GEOSContextHandle_t GetContext();

  /**
   * Calculate the instersection between two lines defined by 4 points.
   *
   * @param[in]   p1   First point of line segment 1.
   * @param[in]   p2   Second point of line segment 1.
   * @param[in]   p3   First point of line segment 2.
   * @param[in]   p4   Second point of line segment 2.
   * @param[out]  c    The intersection point.
   *
   * @returns True if the lines are not parallel.
   */
  bool LineIntersection(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3,
      const cv::Vec2d& p4,
      cv::Vec2d& c);

  /**
   * Calculate the instersection between two line segments defined by 4 points.
   *
   * In the case of parallel overlapping segments, the intersection point
   * closest to p1 is returned.
   *
   * @param[in]   p1   First point of line segment 1.
   * @param[in]   p2   Second point of line segment 1.
   * @param[in]   p3   First point of line segment 2.
   * @param[in]   p4   Second point of line segment 2.
   * @param[out]  c    The intersection point.
   *
   * @returns True if the line segments intersect.
   */
  bool LineSegmentIntersection(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3,
      const cv::Vec2d& p4,
      cv::Vec2d& c);

  /**
   * Check if a point is on a line segment.
   *
   * @param[in]   p1   The point.
   * @param[in]   p2   First point of the line segemnt.
   * @param[in]   p3   Second point of the line segemnt.
   *
   * @returns True if the point is on the line segment.
   */
  bool PointOnLineSegment(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3);

  bool PolygonsIntersect(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b);

  bool PolygonsIntersect(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b,
      GEOSContextHandle_t& ctx);

  double PolygonIntersectionArea(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b);

  double PolygonIntersectionArea(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b,
      GEOSContextHandle_t& ctx);

  void ReleaseContext(GEOSContextHandle_t&  ctx);
}

#endif
