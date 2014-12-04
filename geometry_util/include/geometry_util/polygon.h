// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Richard Garcia <rgarcia@swri.org> (210) 522-3786
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

#ifndef POLYGON_H_
#define POLYGON_H_

#include <sstream>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174532925
#endif

namespace geometry_util
{
  //structure for defining the vertices of a polygon
  typedef struct
  {
    //vertices
    double *x;
    double *y;
  }PolygonD;

  typedef struct
  {
    //vertex
    double x;
    double y;
  }Vertex;

  class Polygon{
  public:

    Polygon();
    Polygon(const Polygon & other);
    Polygon & operator= (const Polygon & other);

    Polygon(double Xs[], double Ys[], int numVertx);

    bool VertexInPolygon(Vertex vertex);

    double* GetXVerticies();

    double* GetYVerticies();

    double GetXVerticie(int num);

    double GetYVerticie(int num);

    int GetNumVerticies();

    bool LineOverlapsPolygon(Vertex start, Vertex end);

    ~Polygon();

  private:

    Vertex FindLineIntersectLine(Vertex start1, Vertex end1, Vertex start2,
        Vertex end2);

    PolygonD _shape;  //list of polygon vertices
    int _nvert;   //number of vertices in this polygon
  };
}  // end namespace geometry_util
#endif /* POLYGON_H_ */
