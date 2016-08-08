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

#ifndef IMAGE_UTIL_DRAW_UTIL_H_
#define IMAGE_UTIL_DRAW_UTIL_H_

#include <string>

#include <opencv2/core/core.hpp>

namespace swri_image_util
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

