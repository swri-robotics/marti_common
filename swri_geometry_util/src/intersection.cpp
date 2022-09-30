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

#include <stdint.h>
#include <stdio.h>  /* for printf */
#include <stdarg.h> /* for va_list */

#include <swri_geometry_util/geometry_util.h>
#include <swri_geometry_util/intersection.h>
#include "util.hpp"

#define HAVE_INT64_T_64  # Prevents conflict with OpenCV typedef of int64
#include <geos_c.h>
#undef HAVE_INT64_T_64

namespace swri_geometry_util
{
  static void geos_msg_handler(const char* fmt, ...)
  {
    va_list ap;
    va_start(ap, fmt);
    vprintf (fmt, ap);
    va_end(ap);
  }

  GEOSContextHandle_t GetContext()
  {
    GEOSContextHandle_t ctx = GEOS_init_r();
    GEOSContext_setNoticeHandler_r(ctx, geos_msg_handler);
    GEOSContext_setErrorHandler_r(ctx, geos_msg_handler);
    return ctx;
  }

  bool LineIntersection(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3,
      const cv::Vec2d& p4,
      cv::Vec2d& c)
  {
    double d = (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]);
    if (d == 0)
    {
      return false;
    }

    double n_a = (p1[0] * p2[1] - p1[1] * p2[0]) * (p3[0] - p4[0]) - (p1[0] - p2[0]) * (p3[0] * p4[1] - p3[1] * p4[0]);
    double n_b = (p1[0] * p2[1] - p1[1] * p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] * p4[1] - p3[1] * p4[0]);

    c[0] = n_a / d;
    c[1] = n_b / d;

    return true;
  }

  bool LineSegmentIntersection(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3,
      const cv::Vec2d& p4,
      cv::Vec2d& c)
  {
    // See: "Intersection of two lines in three-space"
    //      by Ronald Goldman from Graphics Gems

    // Handle case of singe points.
    if (p1 == p2)
    {
      if (PointOnLineSegment(p1, p3, p4))
      {
        c = p1;
        return true;
      }

      return false;
    }
    else if (p3 == p4)
    {
      if (PointOnLineSegment(p3, p1, p2))
      {
        c = p3;
        return true;
      }

      return false;
    }

    cv::Point2d p(p1);
    cv::Point2d r(p2 - p1);
    cv::Point2d q(p3);
    cv::Point2d s(p4 - p3);
    cv::Point2d qp(q - p);

    double rs = r.cross(s);
    if (std::fabs(rs) > std::numeric_limits<float>::epsilon())
    {
      // Explicitly divide components since cv::Point doesn't support division
      // in indigo.
      double t = qp.cross(cv::Point2d(s.x / rs, s.y / rs));
      double u = (qp * -1).cross(cv::Point2d(r.x / -rs, r.y / -rs));

      if (u >= 0 && t >= 0 && u <= 1.0 && t <= 1.0)
      {
        // The lines intersect within the line segments.
        c = p + t * r;
        return true;
      }
      else
      {
        // The lines intersect, but outside the line segments.
        return false;
      }
    }
    else if (std::fabs(qp.cross(r)) > std::numeric_limits<float>::epsilon())
    {
      // The lines are parellel and non-intersecting.
      return false;
    }
    else
    {
      // The lines are parellel and coincident.
      double rlen = r.dot(r);
      cv::Point2d unit_r(r.x / rlen, r.y / rlen);
      double t0 = qp.dot(unit_r);
      double t1 = t0 + s.dot(unit_r);

      if (t0 > t1)
      {
        std::swap(t0, t1);
      }

      if (t0 <= 1.0 && t1 >= 0.0)
      {
        // The line segments overlap.

        double t = std::max(0.0, t0);
        c = p + t * r;
        return true;
      }

      // The line segments don't overlap.
      return false;
    }
  }

  bool PointOnLineSegment(
      const cv::Vec2d& p1,
      const cv::Vec2d& p2,
      const cv::Vec2d& p3)
  {
    // Check if the points are collinear.
    if (((p2[0] - p1[0]) * (p3[1] - p1[0])) - ((p3[0] - p1[0]) * (p2[1] - p1[1])) > std::numeric_limits<float>::epsilon())
    {
      return false;
    }

    if (p2[0] != p3[0])
    {
      return (p1[0] <= p3[0] && p2[0] <= p1[0]) || (p1[0] <= p2[0] && p3[0] <= p1[0]);
    }
    else
    {
      return (p1[1] <= p3[1] && p2[1] <= p1[1]) || (p1[1] <= p2[1] && p3[1] <= p1[1]);
    }
  }

  bool PolygonsIntersect(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b)
  {
    if (a.size() < 3 || b.size() < 3)
    {
      return 0;
    }
    initGEOS(geos_msg_handler, geos_msg_handler);

    // Create GEOS polygon from vertices in vector a.
    GEOSGeometry* a_polygon = VectorToPolygon(a);
    GEOSNormalize(a_polygon);
    GEOSGeometry* b_polygon = VectorToPolygon(b);
    GEOSNormalize(b_polygon);

    bool intersects = (GEOSIntersects(a_polygon, b_polygon) == 1);

    // Free polygon objects.
    GEOSGeom_destroy(a_polygon);
    GEOSGeom_destroy(b_polygon);

    finishGEOS();
    return intersects;
  }

  bool PolygonsIntersect(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b,
      GEOSContextHandle_t& ctx)
  {
    if (a.size() < 3 || b.size() < 3)
    {
      return 0;
    }

    // Create GEOS polygon from vertices in vector a.
    GEOSGeometry* a_polygon = VectorToPolygon(a, ctx);
    GEOSNormalize_r(ctx, a_polygon);
    GEOSGeometry* b_polygon = VectorToPolygon(b, ctx);
    GEOSNormalize_r(ctx, b_polygon);

    bool intersects = (GEOSIntersects_r(ctx, a_polygon, b_polygon) == 1);

    // Free polygon objects.
    GEOSGeom_destroy_r(ctx, a_polygon);
    GEOSGeom_destroy_r(ctx, b_polygon);

    return intersects;
  }

  double PolygonIntersectionArea(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b)
  {
    if (a.size() < 3 || b.size() < 3)
    {
      return 0;
    }

    initGEOS(geos_msg_handler, geos_msg_handler);
    double area = 0;
    GEOSGeometry* a_polygon = VectorToPolygon(a);
    GEOSNormalize(a_polygon);
    GEOSGeometry* b_polygon = VectorToPolygon(b);
    GEOSNormalize(b_polygon);

    GEOSGeometry* intersection = GEOSIntersection(a_polygon, b_polygon);

    if (intersection != 0)
    {
      // Returns 1 on success, 0 on exception
      if (GEOSArea(intersection, &area) == 0)
      {
        area = 0;
      }
    }
    // Free polygon objects
    GEOSGeom_destroy(a_polygon);
    GEOSGeom_destroy(b_polygon);
    GEOSGeom_destroy(intersection);

    finishGEOS();
    return area;
  }

  double PolygonIntersectionArea(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b,
      GEOSContextHandle_t& ctx)
  {
    if (a.size() < 3 || b.size() < 3)
    {
      return 0;
    }

    double area = 0;
    GEOSGeometry* a_polygon = VectorToPolygon(a, ctx);
    GEOSNormalize_r(ctx, a_polygon);
    GEOSGeometry* b_polygon = VectorToPolygon(b, ctx);
    GEOSNormalize_r(ctx, b_polygon);

    GEOSGeometry* intersection = GEOSIntersection_r(ctx, a_polygon, b_polygon);

    if (intersection != 0)
    {
      // Returns 1 on success, 0 on exception
      if (GEOSArea_r(ctx, intersection, &area) == 0)
      {
        area = 0;
      }
    }
    // Free polygon objects
    GEOSGeom_destroy_r(ctx, a_polygon);
    GEOSGeom_destroy_r(ctx, b_polygon);
    GEOSGeom_destroy_r(ctx, intersection);

    return area;
  }

  void ReleaseContext(GEOSContextHandle_t& ctx)
  {
    GEOS_finish_r(ctx);
  }

  GEOSGeometry* VectorToPolygon(const std::vector<cv::Vec2d>& v)
  {
    // Create GEOS polygon from vector of verticies. Allocate one extra
    // element so first and last coordinate are the same, closing the polygon. Assumes
    // calling function has already created context.
    GEOSCoordSequence* coords = GEOSCoordSeq_create(v.size() + 1, 2);

    for (size_t i = 0; i < v.size(); i++)
    {
      GEOSCoordSeq_setX(coords, i, v.at(i)[0]);
      GEOSCoordSeq_setY(coords, i, v.at(i)[1]);
    }

    // Make the first and last coordinate the same to define a closed polygon
    GEOSCoordSeq_setX(coords, v.size(), v.front()[0]);
    GEOSCoordSeq_setY(coords, v.size(), v.front()[1]);


    GEOSGeometry* ring = GEOSGeom_createLinearRing(coords);
    GEOSGeometry* polygon = GEOSGeom_createPolygon(ring, 0, 0);
    GEOSNormalize(polygon);

    return polygon;
  }

  GEOSGeometry* VectorToPolygon(
    const std::vector<cv::Vec2d>& v,
    GEOSContextHandle_t& ctx)
  {
    // Create GEOS polygon from vector of verticies. Allocate one extra
    // element so first and last coordinate are the same, closing the polygon. Assumes
    // calling function has already created context.
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(ctx, v.size() + 1, 2);

    for (size_t i = 0; i < v.size(); i++)
    {
      GEOSCoordSeq_setX_r(ctx, coords, i, v.at(i)[0]);
      GEOSCoordSeq_setY_r(ctx, coords, i, v.at(i)[1]);
    }

    // Make the first and last coordinate the same to define a closed polygon
    GEOSCoordSeq_setX_r(ctx, coords, v.size(), v.front()[0]);
    GEOSCoordSeq_setY_r(ctx, coords, v.size(), v.front()[1]);


    GEOSGeometry* ring = GEOSGeom_createLinearRing_r(ctx, coords);
    GEOSGeometry* polygon = GEOSGeom_createPolygon_r(ctx, ring, 0, 0);
    GEOSNormalize_r(ctx, polygon);

    return polygon;
  }
}
