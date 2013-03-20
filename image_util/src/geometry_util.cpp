// *****************************************************************************
//
// Copyright (C) 2012 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
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

#include <image_util/geometry_util.h>

#include <QPolygonF>
#include <QPointF>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_util
{
  bool Intersects(const BoundingBox& box1, const BoundingBox& box2)
  {
    // Create points for each corner of box 1
    cv::Point_<double> n1(box1.x, box1.y);
    cv::Point_<double> n2(box1.x + box1.width, box1.y + box1.height);
    cv::Point_<double> n3(box1.x, box1.y + box1.height);
    cv::Point_<double> n4(box1.x + box1.width, box1.y);

    // Create points for each corner of box 2
    cv::Point_<double> m1(box2.x, box2.y);
    cv::Point_<double> m2(box2.x + box2.width, box2.y + box2.height);
    cv::Point_<double> m3(box2.x, box2.y + box2.height);
    cv::Point_<double> m4(box2.x + box2.width, box2.y);

    // Return true if any of the corners of one box are contained within the
    // other box.
    return box1.contains(m1) ||
           box1.contains(m2) ||
           box1.contains(m3) ||
           box1.contains(m4) ||
           box2.contains(n1) ||
           box2.contains(n2) ||
           box2.contains(n3) ||
           box2.contains(n4);
  }
  
  double GetOverlappingArea(const cv::Rect& rect,const cv::Mat& rigid_transform)
  {
    // List of points corresponding to the input rectangle.
    std::vector<cv::Vec2f> points;

    // List of points correspondng to the transformed rectangle.
    std::vector<cv::Vec2f> points_t;

    // Create a point for each corner of the input rectangle.
    points.push_back(cv::Vec2f(0,0));
    points.push_back(cv::Vec2f(rect.width, 0));
    points.push_back(cv::Vec2f(rect.width, rect.height));
    points.push_back(cv::Vec2f(0, rect.height));

    // Transform the input points to the transformed points using the rigid
    // transform.
    cv::transform(cv::InputArray(points), cv::OutputArray(points_t), rigid_transform);

    // Use the QPolygon object to get the intersecting area of the input
    // rectangle and the transformed rectangle.

    // Build the polygon corresponding to the input rectangle.
    QPolygonF polygon;
    polygon << QPointF(points[0][0], points[0][1]);
    polygon << QPointF(points[1][0], points[1][1]);
    polygon << QPointF(points[2][0], points[2][1]);
    polygon << QPointF(points[3][0], points[3][1]);
    polygon << QPointF(points[0][0], points[0][1]);

    // Build the polygon corresponding to the transformed rectangle.
    QPolygonF transformed_polygon;
    transformed_polygon << QPointF(points_t[0][0], points_t[0][1]);
    transformed_polygon << QPointF(points_t[1][0], points_t[1][1]);
    transformed_polygon << QPointF(points_t[2][0], points_t[2][1]);
    transformed_polygon << QPointF(points_t[3][0], points_t[3][1]);
    transformed_polygon << QPointF(points_t[0][0], points_t[0][1]);

    // Get the polygon representing the intersection of the input rectangle and
    // the transformed rectangle.
    QPolygonF intersection = polygon.intersected(transformed_polygon);

    // If the intersection is empty, then just return 0 area.
    if (intersection.size() == 0) 
    {
      return 0;
    }
    
    // Build an OpenCV contour to measure the area of the intersection.
    std::vector<cv::Point2f> contour;
    for (int i = 0; i < intersection.size(); i++)
    {
      contour.push_back(cv::Point2f(intersection[i].x(), intersection[i].y()));
    }

    // Scale the area based on the scale factor to get the correct value.
    return cv::contourArea(contour);
  }
}
