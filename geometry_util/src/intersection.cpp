// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <stdint.h>

#include <geometry_util/intersection.h>

#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Polygon.h>

namespace geometry_util
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
