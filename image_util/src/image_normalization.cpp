// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
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
/*
 * image_normalization.cpp
 *
 *  Created on: Jan 19, 2012
 *      Author: kkozak
 */

#include <image_util/image_normalization.h>

namespace image_util
{
  void normalize_illumination(cv::Mat NormImage,
                              cv::Mat SourceImage,
                              cv::Mat& DestImage)
  {
    // note that Norm image should be of type CV_32FC1 and scaled between 0 and
    // 1 (but not exactly equal to 0 anywhere)
    cv::Mat tempIm1;
    SourceImage.convertTo(tempIm1, CV_32FC1, 1.0, 0.0);
    cv::divide(tempIm1, NormImage, tempIm1, 1.0);
    cv::Mat firstImage;
    cv::Mat secondImage;
    tempIm1.convertTo(DestImage, CV_8UC1);
  }

  cv::Mat generate_normalization_image(const std::vector<cv::Mat>& image_list)
  {
    cv::Mat norm_image;
    if (image_list.empty())
    {
      return cv::Mat();
    }

    const cv::Mat& rep_im = image_list.front();

    cv::Mat image_sum(cv::Size(rep_im.cols, rep_im.rows), CV_64F);

    for (uint32_t i = 0; i < image_list.size(); i++)
    {
      cv::Mat temp_im;
      image_list[i].convertTo(temp_im, CV_64F, 1.0, 0.0);

      image_sum = image_sum + temp_im;
    }

    image_sum = image_sum / static_cast<double>(image_list.size());

    cv::Mat mean_image;
    image_sum.convertTo(mean_image, CV_8U);
    cv::Mat temp_norm_image;

    cv::medianBlur(mean_image, temp_norm_image, 45);

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
                     norm_image,
                     cv::Size(25, 25),
                     5,
                     5);

    return norm_image;
  }
  
  void ContrastStretch(
    int32_t grid_size, 
    const cv::Mat& source_image,
    cv::Mat& dest_image)
  {   
    int x_bin_w = std::ceil(static_cast<double>(source_image.cols) / grid_size);
    int y_bin_h = std::ceil(static_cast<double>(source_image.rows) / grid_size);
    
    cv::Mat max_vals(grid_size + 1, grid_size + 1, CV_64F);
    cv::Mat min_vals(grid_size + 1, grid_size + 1, CV_64F);
    
    for(int i = 0; i < grid_size + 1; i++)
    {
      for(int j = 0; j < grid_size + 1; j++)
      {
        double minVal, maxVal;
        
        cv::Rect roi = cv::Rect(j * x_bin_w  - x_bin_w / 2,
                       i * y_bin_h - y_bin_h / 2, x_bin_w, y_bin_h);
        roi.x = std::max(0, roi.x);
        roi.y = std::max(0, roi.y);
        roi.width = std::min(source_image.cols - roi.x, roi.width);
        roi.height = std::min(source_image.rows - roi.y, roi.height);
                       
        cv::minMaxLoc(source_image(roi), &minVal, &maxVal, 0, 0);
        max_vals.at<double>(i, j) = maxVal;
        min_vals.at<double>(i, j) = minVal;
      }
    }

    // Stretch contrast accordingly
    for(int i = 0; i < source_image.rows; i++)
    {
      int ii = i / y_bin_h;
      double py = (i - ii * y_bin_h) / ((double) y_bin_h);

      for(int j = 0; j < source_image.cols; j++)
      {
        // Find relevant min and max values
        int jj = j / x_bin_w;

        // Stretch histogram
        //double minVal = min_vals.at<double>(ii, jj);
        double maxVal = max_vals.at<double>(ii, jj);

        // interp x
        double px = (j - jj * x_bin_w) / ((double) x_bin_w);

        //4-point interpolation
        double xM1 = maxVal + px * (max_vals.at<double>(ii, jj+1) - maxVal);
        double xM2 = max_vals.at<double>(ii+1, jj) + px * (max_vals.at<double>(ii+1, jj+1) - max_vals.at<double>(ii+1, jj));
        double M = xM1 + py * (xM2 - xM1);

        //double xm1 = minVal + px*(min_vals.at<double>(ii, jj+1) - minVal);
        //double xm2 = min_vals.at<double>(ii+1, jj) + px*(min_vals.at<double>(ii+1, jj+1) - min_vals.at<double>(ii+1, jj));
        //double m = xm1 + py*(xm2 - xm1);
        //minVal = m;
        maxVal = M;

        if(maxVal > 255) maxVal = 255;
        //if(minVal < 0) minVal = 0;
        
        //minVal = 0; // Care about bringing up brightness rather than full contrast stretching?

        // TODO: put a bound on maxVal (we don't want to stretch a value of 20 all the way to 255)
        // (same applies to minVal if we aren't setting to 0 above

        double val = source_image.at<uint8_t>(i,j);
        val = val * 255.0 / maxVal;
        if(val > 255) val = 255;
        if(val < 0) val = 0;
        dest_image.at<uint8_t>(i,j) = val;
      }
    }
  }
}
