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

#include <image_util/rolling_normalization.h>

namespace image_util
{
  RollingNormalization::RollingNormalization(int32_t size) :
    max_size_(size),
    samples_(0)
  {
    
  }
  
  RollingNormalization::~RollingNormalization()
  {
  
  }
    
  cv::Mat RollingNormalization::AddSample(const cv::Mat& image)
  {
    if (samples_ == 0)
    {
      image.convertTo(average_image_, CV_64F, 1.0, 0.0);
    }
    else
    {
      cv::Mat temp;
      image.convertTo(temp, CV_64F, 1.0, 0.0);
      double s = static_cast<double>(samples_);
      average_image_ = (average_image_ * s + temp) / (s + 1.0);
    }
    
    samples_++;
    if (samples_ > max_size_)
    {
      samples_ = max_size_;
    }
    
    cv::Mat mean_image;
    average_image_.convertTo(mean_image, CV_8U);
    cv::Mat temp_norm_image;

    cv::medianBlur(mean_image, temp_norm_image, 25);

    cv::Mat temp_norm_image2;
    temp_norm_image.convertTo(temp_norm_image2, CV_32F);
    double max1 = 0;
    for (int32_t i = 0; i < temp_norm_image2.rows; i++)
    {
      for (int32_t j = 0; j < temp_norm_image2.cols; j++)
      {
        if (temp_norm_image2.at<float>(i, j) > max1)
        {
          max1 = temp_norm_image2.at<float>(i, j);
        }
      }
    }

    temp_norm_image2 = temp_norm_image2 * (255.0 / max1);

    cv::Mat temp_norm_image3;
    temp_norm_image2.convertTo(temp_norm_image3, CV_8U, 1.0, 0.0);


    cv::GaussianBlur(temp_norm_image3,
                     norm_image_,
                     cv::Size(15, 15),
                     5,
                     5);
    return norm_image_;
  }
}
