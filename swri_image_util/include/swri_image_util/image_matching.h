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

#ifndef IMAGE_UTIL_IMAGE_MATCHING_H_
#define IMAGE_UTIL_IMAGE_MATCHING_H_

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace swri_image_util
{
  /**
   * @brief      Computes the fundamental matrix for a set of matching points in
   *             two different images.  The method also returns the inlier
   *             keypoints for both frames.
   *
   * @param[in]  points1              The keypoints for the first image
   * @param[in]  points2              The matching keypoints for a second image
   * @param[out] fundamental_matrix   The computed fundamental matrix
   * @param[out] inliers1             The inlier keypoints from the first set
   * @param[out] inliers2             The inlier keypoints from the second set
   * @param[in]  max_distance         The maximum allowable distance (in pixels)
   *                                  from the computed epipolar line
   * @param[in]  confidence           The confidence level (which affects the
   *                                  number of iterations)
   */
  void GetFundamentalInliers(const cv::Mat points1,
                             const cv::Mat points2,
                             cv::Mat& fundamental_matrix,
                             cv::Mat& inliers1,
                             cv::Mat& inliers2,
                             double max_distance = 1.0,
                             double confidence = 0.99);

  /**
   * @brief      Computes the fundamental matrix for a set of matching points in
   *             two different images.  The method also returns the inlier
   *             keypoints for both frames.
   *
   * @param[in]  points1              The keypoints for the first image
   * @param[in]  points2              The matching keypoints for a second image
   * @param[out] fundamental_matrix   The computed fundamental matrix
   * @param[out] inliers1             The inlier keypoints from the first set
   * @param[out] inliers2             The inlier keypoints from the second set
   * @param[out] indices              The indices of the inlier keypoints
   * @param[in]  max_distance         The maximum allowable distance (in pixels)
   *                                  from the computed epipolar line
   * @param[in]  confidence           The confidence level (which affects the
   *                                  number of iterations)
   */
  void GetFundamentalInliers(const cv::Mat points1,
                             const cv::Mat points2,
                             cv::Mat& fundamental_matrix,
                             cv::Mat& inliers1,
                             cv::Mat& inliers2,
                             std::vector<uint32_t>& indices,
                             double max_distance = 1.0,
                             double confidence = 0.99);


  /**
   * @brief      Converts keypoints and matches into two cv::Mats in which the
   *             the matching keypoints from kp1 and kp2 are ordered in the same
   *             manner
   *
   * @param[in]  kp1        The first set of keypoints
   * @param[in]  kp2        The second set of keypoints
   * @param[in]  matches    The vector of matches
   * @param[out] kp1_out    The first set of matching keypoints output as a Mat
   * @param[out] kp2_out    The second set of matching keypoints output as a Mat
   */
  void ConvertMatches(const std::vector<cv::KeyPoint>& kp1,
                      const std::vector<cv::KeyPoint>& kp2,
                      const std::vector<cv::DMatch>& matches,
                      cv::Mat& kp1_out,
                      cv::Mat& kp2_out);
}

#endif  // IMAGE_UTIL_IMAGE_MATCHING_H_

