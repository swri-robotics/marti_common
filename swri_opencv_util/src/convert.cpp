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

#include <swri_opencv_util/convert.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace swri_opencv_util
{
  cv::Mat ToBgra8(
      const cv::Mat& mat,
      const cv::Mat& mask,
      bool is_rgb,
      double a,
      double b)
  {
    if (mat.empty())
    {
      return mat;
    }

    cv::Mat scaled;

    // Autoscale if a zero
    if(a == 0)
    {
      double min, max;
      cv::minMaxLoc(mat, &min, &max, 0, 0, mask);

      if(mat.type() == CV_8UC1)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8U, a, b);

        cv::Mat color;
        cv::cvtColor(scaled, color, cv::COLOR_GRAY2BGRA);
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else if(mat.type() == CV_32FC1)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8U, a, b);
 
        cv::Mat color;
        cv::cvtColor(scaled, color, cv::COLOR_GRAY2BGRA);
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else if(mat.type() == CV_32FC3)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8UC3, a, b);

        cv::Mat color;
        
        if (is_rgb)
        {
          cv::cvtColor(scaled, color, cv::COLOR_RGB2BGRA);
        }
        else
        {
          cv::cvtColor(scaled, color, cv::COLOR_BGR2BGRA);
        }
        
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else if(mat.type() == CV_8UC3)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8UC3, a, b);

        cv::Mat color;
        
        if (is_rgb)
        {
          cv::cvtColor(scaled, color, cv::COLOR_RGB2BGRA);
        }
        else
        {
          cv::cvtColor(scaled, color, cv::COLOR_BGR2BGRA);
        }
        
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else if(mat.type() == CV_8UC4)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8UC4, a, b);

        cv::Mat color;

        if (is_rgb)
        {
          cv::cvtColor(scaled, color, cv::COLOR_RGBA2BGRA);
        }
        else
        {
          color = scaled;
        }

        SetAlpha(color, 255);
        if (!mask.empty())
        {
          color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        }
        
        scaled = color;
      }
    }
    else
    {
      if(mat.type() == CV_8UC3)
      {
        mat.convertTo(scaled, CV_8UC3, a, b);
        
        cv::Mat color;
        if (is_rgb)
        {
          cv::cvtColor(scaled, color, cv::COLOR_RGB2BGRA);
        }
        else
        {
          cv::cvtColor(scaled, color, cv::COLOR_BGR2BGRA);
        }
        
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else if(mat.type() == CV_8UC4)
      {
        mat.convertTo(scaled, CV_8UC4, a, b);
        
        cv::Mat color;
        if (is_rgb)
        {
          cv::cvtColor(scaled, color, cv::COLOR_RGBA2BGRA);
        }
        else
        {
          color = scaled;
        }

        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
      else
      {
        mat.convertTo(scaled, CV_8U, a, b);

        cv::Mat color;
        cv::cvtColor(scaled, color, cv::COLOR_GRAY2BGRA);
        SetAlpha(color, 255);
        color.setTo(cv::Scalar(0, 0, 0, 0), mask == 0);
        scaled = color;
      }
    }
    
    return scaled;
  }
  
  void SetAlpha(cv::Mat& mat, uint8_t alpha)
  {
    if (mat.type() == CV_8UC4)
    {
      for (int r = 0; r < mat.rows; r++)
      {
        for (int c = 0; c < mat.cols; c++)
        {
          mat.at<cv::Vec4b>(r, c)[3] = alpha;
        }
      }
    }
  }
}

