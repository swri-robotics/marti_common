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

#ifndef OPENCV_UTIL_MODELS_H_
#define OPENCV_UTIL_MODELS_H_

#include <vector>

#include <opencv2/core/core.hpp>

namespace opencv_util
{

  class Homography
  {
  public:
    typedef cv::Vec4f T;
    typedef cv::Mat M;
    enum { MIN_SIZE = 4 };
    
    static bool GetModel(const std::vector<T>& data, M& model);
    static double GetError(const T& data, const M& model);
  };

  class AffineTransform2d
  {
  public:
    typedef cv::Vec4f T;
    typedef cv::Mat M;
    enum { MIN_SIZE = 3 };
    
    static bool GetModel(const std::vector<T>& data, M& model);
    static double GetError(const T& data, const M& model);
  };

  class RigidTransform2d
  {
  public:
    typedef cv::Vec4f T;
    typedef cv::Mat M;
    enum { MIN_SIZE = 2 };
    
    static bool GetModel(const std::vector<T>& data, M& model);
    static double GetError(const T& data, const M& model);
  };
  
  class Translation2d
  {
  public:
    typedef cv::Vec4f T;
    typedef cv::Mat M;
    enum { MIN_SIZE = 1 };
    
    static bool GetModel(const std::vector<T>& data, M& model);
    static double GetError(const T& data, const M& model);
  };
  
  bool Valid2dPointCorrespondences(
    const cv::Mat& points1, 
    const cv::Mat& points2);
  
  bool ConvertToVec4f(
    const cv::Mat& points1,
    const cv::Mat& points2,
    std::vector<cv::Vec4f>& matched_points);
}

#endif  // OPENCV_UTIL_MODELS_H_
