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
  void RandomColor(int32_t seed, double& r, double& g, double& b);

  /**
   * @brief Map a scalar value to a color gradient.
   *
   * Return a color gradient RGB value by mapping an input value to a specified
   * scale.
   *
   * @param[out] r Red channel of the output gradient color.
   * @param[out] g Green channel of the output gradient color.
   * @param[out] b Blue channel of the output gradient color.
   * @param[in] value The input value to be mapped to a color.
   * @param[in] min The minimum value on the gradient scale.
   * @param[in] max The maximum value on the gradient scale.
   */
  void JetColorMap(
      unsigned char &r,
      unsigned char &g,
      unsigned char &b,
      float value,
      float min,
      float max);

  void DrawOverlap(
      const std::string& title,
      const cv::Mat& image1,
      const cv::Mat& image2,
      const cv::Mat& transform);

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

