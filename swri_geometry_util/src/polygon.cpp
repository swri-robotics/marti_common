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

#include <swri_geometry_util/polygon.h>

namespace swri_geometry_util
{
  //Constructor - create and undefined polygon
  Polygon::Polygon(){
    this->_nvert = 0;
    this->_shape.x = NULL;
    this->_shape.y = NULL;
  }

  //Constructor - create a duplicate polygon
  Polygon::Polygon(const Polygon & other)
  {
    this->_shape.x = new double[other._nvert];
    this->_shape.y = new double[other._nvert];
    this->_nvert = other._nvert;

    for(int i=0; i<other._nvert; i++){
      this->_shape.x[i] = other._shape.x[i];
      this->_shape.y[i] = other._shape.y[i];
    }
  }

  //Operate overload for assign a polygon
  Polygon & Polygon::operator =(const Polygon & other)
  {
    if(this != &other) // protect against invalid self-assignment
    {
      if (this->_nvert > 0)
      {
        delete[] this->_shape.x;
        this->_shape.x = NULL;
        delete[] this->_shape.y;
        this->_shape.y = NULL;
      }
      this->_shape.x = new double[other._nvert];
      this->_shape.y = new double[other._nvert];
      this->_nvert = other._nvert;

      for(int i=0; i<other._nvert; i++){
        this->_shape.x[i] = other._shape.x[i];
        this->_shape.y[i] = other._shape.y[i];
      }
    }
    return *this;
  }

  //Constructor - create a polygon using a list of vertices
  //Assumptions - vertices are in CW order
  Polygon::Polygon(double Xs[], double Ys[], int numVertx){

    this->_shape.x = new double[numVertx];
    this->_shape.y = new double[numVertx];
    this->_nvert = numVertx;

    for(int i=0;i<numVertx;i++)
    {
      this->_shape.x[i] = Xs[i];
      this->_shape.y[i] = Ys[i];
    }
  }

  //Determine if a given vertex lies within this polygon
  //Returns:  True if "vertex" is within this polygon, False otherwise
  bool Polygon::VertexInPolygon(Vertex vertex)
  {
    int i, j, c = 0;
    for (i = 0, j = _nvert-1; i < _nvert; j = i++)
    {
        if (((_shape.y[i]>vertex.y) != (_shape.y[j]>vertex.y)) && (vertex.x <
            (_shape.x[j]-_shape.x[i]) * (vertex.y-_shape.y[i]) /
            (_shape.y[j]-_shape.y[i]) + _shape.x[i]))
            c = !c;
    }
    return c;
  }

  //Determine if a given line segment intersects with or lies within this polygon
  //Returns:  True if line segment defined by "start" and "end" intersects or
  //          lies within this polygon, False otherwise
  bool Polygon::LineOverlapsPolygon(Vertex start, Vertex end)
  {
    Vertex pStart,pEnd, intersect;

    //check if either end point is within the polygon
    if (VertexInPolygon(start) || VertexInPolygon(end))
    {
      return true;
    }

    //check for line intersection with the polygon
    for(int i=0;i < _nvert;i++)
    {
      pStart.x = _shape.x[i];
      pStart.y = _shape.y[i];
      pEnd.x = _shape.x[(i+1)%_nvert];
      pEnd.y = _shape.y[(i+1)%_nvert];

      intersect = FindLineIntersectLine(pStart,pEnd,start,end);
      if(intersect.x != -999.0 && intersect.y != -999.0)//intersection found
      {
        return true;
      }
    }

    return false;
  }

  //Private Function
  //Determines if two line segments intersect
  //Returns:  True if line segments intersect, False otherwise
  Vertex Polygon::FindLineIntersectLine(Vertex start1, Vertex end1,
      Vertex start2, Vertex end2)
  {
    Vertex result;
    result.x = -999.0;
    result.y = -999.0;

    double denom = ((end1.x - start1.x) * (end2.y - start2.y)) -
        ((end1.y - start1.y) * (end2.x - start2.x));

    //no intersection (lines are parallel)
    if (denom == 0)
            return result;

    double numer = ((start1.y - start2.y) * (end2.x - start2.x)) -
        ((start1.x - start2.x) * (end2.y - start2.y));

    double r = numer / denom;

    double numer2 = ((start1.y - start2.y) * (end1.x - start1.x)) -
        ((start1.x - start2.x) * (end1.y - start1.y));

    double s = numer2 / denom;

    //no intersection
    if ((r < 0 || r > 1) || (s < 0 || s > 1))
            return result;

    // Find intersection point
    result.x = start1.x + (r * (end1.x - start1.x));
    result.y = start1.y + (r * (end1.y - start1.y));

    return result;
  }

  //returns all x vertices for this polygon
  double* Polygon::GetXVerticies()
  {
    return this->_shape.x;
  }

  //returns all y vertices for this polygon
  double* Polygon::GetYVerticies()
  {
    return this->_shape.y;
  }

  //returns a specific x vertex
  double Polygon::GetXVerticie(int num)
  {
    return this->_shape.x[num];
  }

  //returns a specific y vertex
  double Polygon::GetYVerticie(int num)
  {
    return this->_shape.y[num];
  }

  int Polygon::GetNumVerticies()
  {
    return this->_nvert;
  }

  //Destructor
  Polygon::~Polygon() {
    if(_shape.x){
      delete[] _shape.x;
      _shape.x = NULL;
    }
    if(_shape.y){
      delete[] _shape.y;
      _shape.y = NULL;
    }
  }
}  // end namespace swri_geometry_util
