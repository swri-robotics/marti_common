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

#ifndef IMAGE_UTIL_GEOMETRY_UTIL_H_
#define IMAGE_UTIL_GEOMETRY_UTIL_H_

#include <vector>

#include <opencv2/core/core.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Vector3.h>

namespace swri_image_util
{
  typedef cv::Rect_<double> BoundingBox;

  /**
   * Calculate the overlapping area of a rectangle an a rigidly transformed
   * version of itself.
   *
   * @param[in]  rect             The rectangle.
   * @param[in]  rigid_transform  The rigid transform.
   *
   * @returns The area of intersection of the two rectangles.
   */
  double GetOverlappingArea(
      const cv::Rect& rect,
      const cv::Mat& rigid_transform);

  /**
   * Determine if two aligned rectangles intersect one another.
   *
   * @param[in]  box1  The first rectangle.
   * @param[in]  box2  The second rectangle.
   *
   * @returns True if box1 intersects with box2.  False otherwise.
   */
  bool Intersects(const BoundingBox& box1, const BoundingBox& box2);

  /**
   * Projects a 3D ellipsoid to an ellipse on the XY-plane.
   *
   * @param[in]  ellipsoid  The ellipsoid represented as a 3x3 float matrix.
   *
   * @returns The ellipse as a 2x2 float matrix if successful.  An empty matrix
   *          otherwise.
   */
  cv::Mat ProjectEllipsoid(const cv::Mat& ellipsiod);

  /**
   * Gets a list of points on the perimeter of an ellipse.
   *
   * @param[in]  ellipse     The ellipse represented as a 2x2 float matrix.
   * @param[in]  center      The center of the ellipse.
   * @param[in]  scale       A scale factor.
   * @param[in]  num_points  The number of points to use.
   *
   * @returns A list of points on the perimeter of the ellipse if successful.
   *          An empty list otherwise.
   */
  std::vector<tf2::Vector3> GetEllipsePoints(
      const cv::Mat& ellipse,
      const tf2::Vector3& center,
      double scale,
      int32_t num_points);
}

#endif  // IMAGE_UTIL_GEOMETRY_UTIL_H_
