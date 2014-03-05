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

#ifndef IMAGE_UTIL_ROLLING_NORMALIZATION_H_
#define IMAGE_UTIL_ROLLING_NORMALIZATION_H_

#include <vector>

// ROS Libraries
#include <ros/ros.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_util
{
  class RollingNormalization
  {
  public:
    RollingNormalization(int32_t size);
    ~RollingNormalization();
    
    cv::Mat AddSample(const cv::Mat& image);
  private:
    int32_t max_size_;
    int32_t samples_;
    cv::Mat average_image_;
    cv::Mat norm_image_;
  };
}


#endif  // IMAGE_UTIL_ROLLING_NORMALIZATION_H_
