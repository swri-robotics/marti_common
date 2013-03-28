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

#include <opencv2/core/core.hpp>

namespace image_util
{
  typedef cv::Rect_<double> BoundingBox;

  double GetOverlappingArea(const cv::Rect& rect, const cv::Mat& rigid_transform);

  bool Intersects(const BoundingBox& box1, const BoundingBox& box2);
}

#endif  // IMAGE_UTIL_GEOMETRY_UTIL_H_
