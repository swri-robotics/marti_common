// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#ifndef OPENCV_UTIL_SHOW_H_
#define OPENCV_UTIL_SHOW_H_

#include <opencv2/core/core.hpp>

namespace opencv_util
{
  void ShowScaled(
      const std::string& name,
      const cv::Mat& mat,
      const cv::Mat& mask = cv::Mat(),
      double a = -1.0, // assume auto-scaling
      double b = 0.0);
}

#endif  // OPENCV_UTIL_SHOW_H_
