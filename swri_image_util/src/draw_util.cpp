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

#include <swri_image_util/draw_util.h>

#include <cstdlib>
#include <algorithm>
#include <random>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <swri_opencv_util/show.h>

namespace swri_image_util
{
  void RandomColor(int32_t seed, double& r, double& g, double& b)
  {
    std::mt19937 mt_entropy(seed);
    std::uniform_real_distribution<double> rand_num_gen(0.0, 1.0);

    r = rand_num_gen(mt_entropy);
    g = rand_num_gen(mt_entropy);
    b = rand_num_gen(mt_entropy);
  }

  void JetColorMap(
      unsigned char &r,
      unsigned char &g,
      unsigned char &b,
      float value,
      float min,
      float max)
  {
    float max4 = (max - min) / 4.0;
    value -= min;

    if (value == HUGE_VAL)
    {
      r = g = b = 255;
    }
    else if (value < 0)
    {
      r = g = b = 0;
    }
    else if (value < max4)
    {
      unsigned char c1 = 144;

      r = 0;
      g = 0;
      b = c1 + (unsigned char) ((255 - c1) * value / max4);
    }
    else if (value < 2 * max4)
    {
      r = 0;
      g = (unsigned char) (255 * (value - max4) / max4);
      b = 255;
    }
    else if (value < 3 * max4)
    {
      r = (unsigned char) (255 * (value - 2 * max4) / max4);
      g = 255;
      b = 255 - r;
    }
    else if (value < max)
    {
      r = 255;
      g = (unsigned char) (255 - 255 * (value - 3 * max4) / max4);
      b = 0;
    }
    else
    {
      r = 255;
      g = 0;
      b = 0;
    }
  }

  void DrawOverlap(
      const std::string& title,
      const cv::Mat& image1,
      const cv::Mat& image2,
      const cv::Mat& transform)
  {
    if (image1.rows == image2.rows && image1.cols == image2.cols)
    {
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
      cv::Mat& image_out,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color,
      bool draw_image_borders)
  {
    cv::Size size(image1.cols + image2.cols, std::max(image1.rows, image2.rows));
    image_out.create(size, CV_MAKETYPE(image1.depth(), 3));
    image_out.setTo(cv::Vec3b(0, 0, 0));
    cv::Mat draw_image1 = image_out(cv::Rect(0, 0, image1.cols, image1.rows));
    cv::Mat draw_image2 = image_out(cv::Rect(image1.cols, 0, image2.cols, image2.rows));

    if (image1.type() == CV_8U)
    {
      cvtColor(image1, draw_image1, cv::COLOR_GRAY2BGR);
    }
    else
    {
      image1.copyTo(draw_image1);
    }

    if (image2.type() == CV_8U)
    {
      cvtColor(image2, draw_image2, cv::COLOR_GRAY2BGR);
    }
    else
    {
      image2.copyTo(draw_image2);
    }

    if (draw_image_borders)
    {
      cv::rectangle(draw_image1,
                    cv::Point(0, 0),
                    cv::Point(image1.cols, image1.rows),
                    cv::Scalar(0, 0, 0),
                    2);

      cv::rectangle(draw_image2,
                    cv::Point(0, 0),
                    cv::Point(image2.cols, image2.rows),
                    cv::Scalar(0, 0, 0),
                    2);
    }

    cv::RNG rng = cv::theRNG();
    bool rand_color = color == cv::Scalar::all(-1);

    for (int i = 0; i < points1.rows; i++)
    {
      cv::Scalar match_color = rand_color ? cv::Scalar(rng(256), rng(256), rng(256)) : color;
      cv::Point2f center1(
        cvRound(points1.at<cv::Vec2f>(0, i)[0] * 16.0),
        cvRound(points1.at<cv::Vec2f>(0, i)[1] * 16.0));
      cv::Point2f center2(
        cvRound(points2.at<cv::Vec2f>(0, i)[0] * 16.0),
        cvRound(points2.at<cv::Vec2f>(0, i)[1] * 16.0));
      cv::Point2f dcenter2(
        std::min(center2.x + draw_image1.cols * 16.0, (image_out.cols - 1) * 16.0), 
        center2.y);
      circle(draw_image1, center1, 48, match_color, 1, cv::LINE_AA, 4);
      circle(draw_image2, center2, 48, match_color, 1, cv::LINE_AA, 4);
      line(image_out, center1, dcenter2, match_color, 1, cv::LINE_AA, 4);
    }
  }

  void DrawMatches(
      const std::string& title,
      const cv::Mat image1,
      const cv::Mat image2,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color,
      bool draw_image_borders)
  {
    cv::Mat image_out;
    DrawMatches(image_out,
                image1,
                image2,
                points1,
                points2,
                color,
                draw_image_borders);

    swri_opencv_util::ShowScaled(title, image_out);
  }

  void DrawMatches(
      const std::string& title,
      const cv::Mat image,
      const cv::Mat points1,
      const cv::Mat points2,
      const cv::Scalar& color1,
      const cv::Scalar& color2,
      bool draw_image_borders)
  {
    cv::Mat draw_image;
    if (image.type() == CV_8U)
    {
      cvtColor(image, draw_image, cv::COLOR_GRAY2BGR);
    }
    else
    {
      draw_image = image.clone();
    }

    for (int i = 0; i < points1.rows; i++)
    {
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
}
