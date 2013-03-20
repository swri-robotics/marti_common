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

#ifndef IMAGE_UTIL_DRAW_UTIL_H_
#define IMAGE_UTIL_DRAW_UTIL_H_

#include <string>

#include <opencv2/core/core.hpp>

namespace image_util
{
  void DrawMatches(
      cv::Mat& image_out,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color = cv::Scalar::all(-1),
      bool draw_image_borders = false);

  void DrawMatches(
      const std::string& title,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color = cv::Scalar::all(-1),
      bool draw_image_borders = false);

  void DrawMatches(
      const std::string& title,
      const cv::Mat image,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color1,
      const cv::Scalar& color2,
      bool draw_image_borders = false);
}

#endif  // IMAGE_UTIL_DRAW_UTIL_H_

