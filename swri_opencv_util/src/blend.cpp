// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#include <swri_opencv_util/blend.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace swri_opencv_util {

cv::Mat blend(
    const cv::Mat& src1,
    const cv::Mat& alpha1,
    const cv::Mat& src2,
    const cv::Mat& alpha2)
{
  int out_type = src1.type();
  cv::Mat s1, s2, a1, a2;
  alpha1.convertTo(a1, CV_32F);
  alpha2.convertTo(a2, CV_32F);
  src1.convertTo(s1, CV_32F);
  src2.convertTo(s2, CV_32F);
  cv::Mat w1;
  cv::divide(a1, a1 + a2, w1);
  cv::Mat w2 = -w1 + 1.0;

  cv::Mat blended = s1.mul(w1) + s2.mul(w2);
  cv::Mat blended_out;
  blended.convertTo(blended_out, out_type);
  return blended_out;
}

cv::Mat blend(
    const cv::Mat& overlay,
    const cv::Mat& base,
    double alpha)
{
  alpha = std::min(1.0, alpha);
  alpha = std::max(0.0, alpha);
  cv::Mat blended;
  cv::addWeighted(overlay, alpha, base, 1.0 - alpha, 0, blended);
  return blended;
}

cv::Mat overlayColor(
    const cv::Mat& src,
    const cv::Mat& mask,
    const cv::Scalar& color,
    double alpha)
{
  alpha = std::min(1.0, alpha);
  alpha = std::max(0.0, alpha);

  cv::Size size = src.size();
  cv::Mat color_image;

  if (src.type() == CV_8U)
  {
    cv::cvtColor(src, color_image, cv::COLOR_GRAY2BGR);
  }
  else if (src.type() == CV_32F || src.type() == CV_16U)
  {
    cv::Mat tmp;
    src.convertTo(tmp, CV_8U);
    cv::cvtColor(tmp, color_image, cv::COLOR_GRAY2BGR);
  }
  else if (src.type() == CV_32FC3 || src.type() == CV_16UC3)
  {
    src.convertTo(color_image, CV_8UC3);
  }
  else if (src.type() != CV_8UC3)
  {
    color_image = src;
  }
  else
  {
    return cv::Mat();
  }

  // Create the color overlay image.
  cv::Mat overlay(size, CV_8UC3);
  overlay.setTo(color);

  // Create the alpha channel for the overlay image.
  cv::Mat overlay_alpha = cv::Mat::zeros(size, CV_32F);
  overlay_alpha.setTo(alpha, mask);

  // Create the alpha channel for the base image.
  cv::Mat base_alpha(size, CV_32F);
  base_alpha = 1.0 - alpha;

  // Blend the images based on the relative alpha values at each pixel.
  return swri_opencv_util::blend(overlay, overlay_alpha, color_image, base_alpha);
}

}

