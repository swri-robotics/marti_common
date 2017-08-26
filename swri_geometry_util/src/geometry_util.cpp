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
      const tf::Vector3& plane_normal,
      const tf::Vector3& plane_point,
      const tf::Vector3& point)
  {
    return plane_normal.normalized().dot(point - plane_point);
  }
  
  double DistanceFromLineSegment(
      const tf::Vector3& line_start,
      const tf::Vector3& line_end,
      const tf::Vector3& point)
  {    
    return point.distance(ProjectToLineSegment(line_start, line_end, point));
  }
  
  tf::Vector3 ProjectToLineSegment(
      const tf::Vector3& line_start,
      const tf::Vector3& line_end,
      const tf::Vector3& point)
  {
    tf::Vector3 v = line_end - line_start;
    tf::Vector3 r = point - line_start;
    
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

  bool ClosestPointToLines(
      const tf::Vector3& a1,
      const tf::Vector3& a2,
      const tf::Vector3& b1,
      const tf::Vector3& b2,
      tf::Vector3& point)
  {
    tf::Vector3 u = a1 - a2;
    tf::Vector3 v = b1 - b2;
    if (u.length() == 0 || v.length() == 0)
    {
      return false;
    }
    tf::Vector3 w = u.cross(v);
    tf::Vector3 s = b1 - a1;
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
    tf::Vector3 x = a1 + u * (s.cross(v).dot(w) / f);
    tf::Vector3 y = b1 + v * (s.cross(u).dot(w) / f);
    point = (x + y) / 2;
    return true;
  }
}
