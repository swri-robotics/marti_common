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

#ifndef IMAGE_UTIL_IMAGE_NORMALIZATION_H_
#define IMAGE_UTIL_IMAGE_NORMALIZATION_H_

#include <vector>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace swri_image_util
{
  /**
   * Normalizes the illumination in an image using a normalization image as a 
   * template
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
   * Normalizes the illumination in an image using contrast stretching.
   *
   * @param[in]  grid_size     The grid size to normalize on
   * @param[in]  source_image  The image to normalize
   * @param[out] dest_image    The resulting normalized image
   */
  void ContrastStretch(
      int32_t grid_size,
      const cv::Mat& source_image,
      cv::Mat& dest_image,
      const cv::Mat& mask=cv::Mat(),
      double max_min = 0.0,
      double min_max = 0.0);

  /** 
   * Normalizes the illumination in an image using approach from OpenCV's
   * stereo block matching.
   */
  void NormalizeResponse(
      const cv::Mat& src, 
      cv::Mat& dst, 
      int winsize, 
      int ftzero, 
      uchar* buf);

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
