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

#ifndef IMAGE_UTIL_IMAGE_NORMALIZATION_H_
#define IMAGE_UTIL_IMAGE_NORMALIZATION_H_

#include <vector>

// ROS Libraries
#include <ros/ros.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_util
{
  /**
   * @brief      Normalizes the illumination in an image using a normalization
   *             image as a template
   *
   * @param[in]  NormImage      A normalization image
   * @param[in]  SourceImage    The image to normalize
   * @param[out] DestImage      The resulting normalized image
   */
  void normalize_illumination(
      cv::Mat NormImage,
      cv::Mat SourceImage,
      cv::Mat& DestImage);


  /**
   * @brief      Computes a best estimate of a normalization image from a vector
   *             of images.
   *
   * @param[in]  image_list   A vector of images -- it is best to have a
   *                          diversity of images in this list.
   *
   * @retval     Returns the normalization image
   */
  cv::Mat generate_normalization_image(const std::vector<cv::Mat>& image_list);

	/**
	 * @brief Convert the input Mat to 8 bit
	 *
	 *
	 * @param[in] image  The input image
	 *
	 * @returns The 8-bit Mat.
	 */
	cv::Mat scale_2_8bit(const cv::Mat& image);
	/**
	 * @brief Convert the input Mat to 8 bit color
	 *
	 *
	 * @param[in] image  The input image
	 *
	 * @returns The 8-bit color Mat.
	 */
	cv::Mat scale_2_8bit_color(const cv::Mat& image);
}


#endif  // IMAGE_UTIL_IMAGE_NORMALIZATION_H_
