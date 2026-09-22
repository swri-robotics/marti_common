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

// The functions here display images with HighGUI, so they are built into
// swri_image_util_display rather than swri_image_util.  See CMakeLists.txt.

#include "swri_image_util/draw_util.h"

#include <string>

#include <opencv2/imgproc/imgproc.hpp>

#include "swri_opencv_util/show.h"

namespace swri_image_util
{
void DrawOverlap(
  const std::string & title,
  const cv::Mat & image1,
  const cv::Mat & image2,
  const cv::Mat & transform)
{
  if (image1.rows == image2.rows && image1.cols == image2.cols) {
    cv::Mat image2_warped;
    cv::warpAffine(
      image2,
      image2_warped,
      transform,
      cv::Size(image2.cols, image2.rows));

    cv::Mat sub = image1 - image2_warped;

    swri_opencv_util::ShowScaled(title, sub);
  }
}

void DrawMatches(
  const std::string & title,
  const cv::Mat image1,
  const cv::Mat image2,
  const cv::Mat points1,
  const cv::Mat points2,
  const cv::Scalar & color,
  bool draw_image_borders)
{
  cv::Mat image_out;
  DrawMatches(
    image_out,
    image1,
    image2,
    points1,
    points2,
    color,
    draw_image_borders);

  swri_opencv_util::ShowScaled(title, image_out);
}

void DrawMatches(
  const std::string & title,
  const cv::Mat image,
  const cv::Mat points1,
  const cv::Mat points2,
  const cv::Scalar & color1,
  const cv::Scalar & color2,
  bool draw_image_borders)
{
  cv::Mat draw_image;
  if (image.type() == CV_8U) {
    cvtColor(image, draw_image, cv::COLOR_GRAY2BGR);
  } else {
    draw_image = image.clone();
  }

  for (int i = 0; i < points1.rows; i++) {
    cv::Point2f center1(
      cvRound(points1.at<cv::Vec2f>(0, i)[0] * 16.0),
      cvRound(points1.at<cv::Vec2f>(0, i)[1] * 16.0));
    cv::Point2f center2(cvRound(
        points2.at<cv::Vec2f>(0, i)[0] * 16.0),
      cvRound(points2.at<cv::Vec2f>(0, i)[1] * 16.0));
    circle(draw_image, center1, 48, color1, 1, cv::LINE_AA, 4);
    line(draw_image, center1, center2, color2, 1, cv::LINE_AA, 4);
  }

  swri_opencv_util::ShowScaled(title, draw_image);
}
}  // namespace swri_image_util
