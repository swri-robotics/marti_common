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

#include <swri_geometry_util/intersection.h>

#define HAVE_INT64_T_64  # Prevents conflict with OpenCV typedef of int64
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Polygon.h>
#undef HAVE_INT64_T_64

namespace swri_geometry_util
{
  bool PolygonsIntersect(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b)
  {
    // Create GEOS polygon from vertices in vector a.
    geos::geom::CoordinateSequence* a_coords = new geos::geom::CoordinateArraySequence();
    for (size_t i = 0; i < a.size(); i++)
    {
      a_coords->add(geos::geom::Coordinate(a[i][0], a[i][1]));
    }
    a_coords->add(a_coords->front());

    geos::geom::LinearRing* a_ring = geos::geom::GeometryFactory::getDefaultInstance()->createLinearRing(a_coords);
    geos::geom::Polygon* a_polygon = geos::geom::GeometryFactory::getDefaultInstance()->createPolygon(a_ring, 0);
    a_polygon->normalize();

    // Create GEOS polygon from vertices in vector b.
    geos::geom::CoordinateSequence* b_coords = new geos::geom::CoordinateArraySequence();
    for (size_t i = 0; i < b.size(); i++)
    {
      b_coords->add(geos::geom::Coordinate(b[i][0], b[i][1]));
    }
    b_coords->add(b_coords->front());

    geos::geom::LinearRing* b_ring = geos::geom::GeometryFactory::getDefaultInstance()->createLinearRing(b_coords);
    geos::geom::Polygon* b_polygon = geos::geom::GeometryFactory::getDefaultInstance()->createPolygon(b_ring, 0);
    b_polygon->normalize();

    bool intersects =  a_polygon->intersects(b_polygon);

    // Free polygon objects.
    delete a_polygon;
    delete b_polygon;

    return intersects;
  }

  double PolygonIntersectionArea(
      const std::vector<cv::Vec2d>& a,
      const std::vector<cv::Vec2d>& b)
  {
    double area = 0;
    // Create GEOS polygon from vertices in vector a.
    geos::geom::CoordinateSequence* a_coords = new geos::geom::CoordinateArraySequence();
    for (size_t i = 0; i < a.size(); i++)
    {
      a_coords->add(geos::geom::Coordinate(a[i][0], a[i][1]));
    }
    a_coords->add(a_coords->front());

    geos::geom::LinearRing* a_ring = geos::geom::GeometryFactory::getDefaultInstance()->createLinearRing(a_coords);
    geos::geom::Polygon* a_polygon = geos::geom::GeometryFactory::getDefaultInstance()->createPolygon(a_ring, 0);
    a_polygon->normalize();

    // Create GEOS polygon from vertices in vector b.
    geos::geom::CoordinateSequence* b_coords = new geos::geom::CoordinateArraySequence();
    for (size_t i = 0; i < b.size(); i++)
    {
      b_coords->add(geos::geom::Coordinate(b[i][0], b[i][1]));
    }
    b_coords->add(b_coords->front());

    geos::geom::LinearRing* b_ring = geos::geom::GeometryFactory::getDefaultInstance()->createLinearRing(b_coords);
    geos::geom::Polygon* b_polygon = geos::geom::GeometryFactory::getDefaultInstance()->createPolygon(b_ring, 0);
    b_polygon->normalize();

    if (a_polygon->intersects(b_polygon))
    {
      area = a_polygon->intersection(b_polygon)->getArea();
    }

    // Free polygon objects.
    delete a_polygon;
    delete b_polygon;

    return area;
  }
}
