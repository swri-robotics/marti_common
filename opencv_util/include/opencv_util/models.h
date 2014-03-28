// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#ifndef OPENCV_UTIL_MODELS_H_
#define OPENCV_UTIL_MODELS_H_

#include <vector>

#include <opencv2/core/core.hpp>

namespace opencv_util
{
  class RigidTransform2d
  {
  public:
    typedef cv::Vec4f T;
    typedef cv::Mat M;
    enum { MIN_SIZE = 3 };
    
    static bool GetModel(const std::vector<T>& data, M& model);
    static double GetError(const T& data, const M& model);
  };
  
  bool ConvertToVec4f(
    const cv::Mat& points1,
    const cv::Mat& points2,
    std::vector<cv::Vec4f>& matched_points);
}

#endif  // OPENCV_UTIL_MODELS_H_
