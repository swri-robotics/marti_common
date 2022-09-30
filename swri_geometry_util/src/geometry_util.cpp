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

#include <swri_geometry_util/geometry_util.h>

namespace swri_geometry_util
{
  double DistanceFromPlane(
      const tf2::Vector3& plane_normal,
      const tf2::Vector3& plane_point,
      const tf2::Vector3& point)
  {
    return plane_normal.normalized().dot(point - plane_point);
  }
  
  tf2::Vector3 ProjectPointToPlane(
      const tf2::Vector3& plane_normal,
      const tf2::Vector3& plane_point,
      const tf2::Vector3& point)
  {
    double d = DistanceFromPlane(plane_normal, plane_point, point);
    return point - plane_normal * d;
  }

  double DistanceFromLineSegment(
      const tf2::Vector3& line_start,
      const tf2::Vector3& line_end,
      const tf2::Vector3& point)
  {    
    return point.distance(ProjectToLineSegment(line_start, line_end, point));
  }
  
  double DistanceFromLineSegment(
      const cv::Vec2d& line_start,
      const cv::Vec2d& line_end,
      const cv::Vec2d& point)
  {
    const cv::Vec2d proj = ProjectToLineSegment(line_start, line_end, point);
    return std::sqrt(
          (point[0] - proj[0]) * (point[0] - proj[0]) +
          (point[1] - proj[1]) * (point[1] - proj[1]));
  }

  tf2::Vector3 ProjectToLineSegment(
      const tf2::Vector3& line_start,
      const tf2::Vector3& line_end,
      const tf2::Vector3& point)
  {
    tf2::Vector3 v = line_end - line_start;
    tf2::Vector3 r = point - line_start;
    
    double t = r.dot(v);
    if (t <= 0)
    {
      return line_start;
    }
    
    double b = v.dot(v);
    if (t >= b)
    {
      return line_end;
    }
    
    return line_start + (t / b) * v;
  }


  cv::Vec2d ProjectToLineSegment(
      const cv::Vec2d& line_start,
      const cv::Vec2d& line_end,
      const cv::Vec2d& point)
  {
    cv::Point2d v(line_end - line_start);
    cv::Point2d r(point - line_start);

    double t = r.dot(v);
    if (t <= 0)
    {
      return line_start;
    }

    double b = v.dot(v);
    if (t >= b)
    {
      return line_end;
    }

    // Explicitly multiply components since cv::Point doesn't support operation
    // in indigo.
    return line_start + cv::Vec2d(v.x * (t / b), v.y * (t / b));
  }

  bool PointInPolygon(
      const std::vector<cv::Vec2d>& polygon,
      const cv::Vec2d& point)
  {
    if (polygon.size() < 2)
    {
      return false;
    }

    bool is_inside = false;
    if (((polygon.front()[1] > point[1]) != (polygon.back()[1] > point[1])) &&
         (point[0] < (polygon.back()[0] - polygon.front()[0]) * (point[1] - polygon.front()[1]) /
                     (polygon.back()[1] - polygon.front()[1]) + polygon.front()[0]))
    {
      is_inside = !is_inside;
    }

    for (size_t i = 1; i < polygon.size(); i++)
    {
      if (((polygon[i][1] > point[1]) != (polygon[i - 1][1] > point[1])) &&
           (point[0] < (polygon[i - 1][0] - polygon[i][0]) * (point[1] - polygon[i][1]) /
                       (polygon[i - 1][1] - polygon[i][1]) + polygon[i][0]))
      {
        is_inside = !is_inside;
      }
    }

    return is_inside;
  }

  double DistanceFromPolygon(
      const std::vector<cv::Vec2d>& polygon,
      const cv::Vec2d& point)
  {
    if (polygon.empty())
    {
      return -1;
    }

    double dist = DistanceFromLineSegment(polygon.front(), polygon.back(), point);
    for (size_t i = 1; i < polygon.size(); i++)
    {
      dist = std::min(dist, DistanceFromLineSegment(polygon[i], polygon[i - 1], point));
    }

    return dist;
  }

  bool ClosestPointToLines(
      const tf2::Vector3& a1,
      const tf2::Vector3& a2,
      const tf2::Vector3& b1,
      const tf2::Vector3& b2,
      tf2::Vector3& point)
  {
    tf2::Vector3 u = a1 - a2;
    tf2::Vector3 v = b1 - b2;
    if (u.length() == 0 || v.length() == 0)
    {
      return false;
    }
    tf2::Vector3 w = u.cross(v);
    tf2::Vector3 s = b1 - a1;
    if (s.length() == 0)
    {
      point = a1;
      return true;
    }
    double f = w.dot(w);
    if (f == 0)
    {
      return false;
    }
    tf2::Vector3 x = a1 + u * (s.cross(v).dot(w) / f);
    tf2::Vector3 y = b1 + v * (s.cross(u).dot(w) / f);
    point = (x + y) / 2;
    return true;
  }

}
