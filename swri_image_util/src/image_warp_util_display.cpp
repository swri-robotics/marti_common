// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

// This overload of EstimateNominalAngle() can display its result with HighGUI,
// so it is built into swri_image_util_display rather than swri_image_util.
// See CMakeLists.txt.

#include "swri_image_util/image_warp_util.h"

#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/logging.hpp>

namespace swri_image_util
{
cv::Mat PitchAndRollEstimator::EstimateNominalAngle(
  double & nominal_pitch,
  double & nominal_roll,
  bool show_image_diff,
  rclcpp::Logger logger)
{
  if (kp1_matched_.empty() || kp2_matched_.empty()) {
    return cv::Mat();
  }

  std::chrono::system_clock::time_point T1 = std::chrono::system_clock::now();
  cv::Mat T_rigid = EstimateNominalAngle(
    kp1_matched_,
    kp2_matched_,
    cv::Size(im1_.cols, im1_.rows),
    nominal_pitch,
    nominal_roll);

  std::chrono::system_clock::time_point T2 = std::chrono::system_clock::now();

  RCLCPP_ERROR(
    logger, "Estimate Nominal Angle time = %g",
    std::chrono::duration_cast<std::chrono::duration<float>>(T2 - T1).count());
  cv::Mat R = GetR(nominal_pitch, nominal_roll);

  if (show_image_diff) {
    // Do the warping and transformation and show the results
    cv::Mat warped_im1;
    cv::Mat warped_im2;

    warper_.warp(im1_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im1);
    warper_.warp(im2_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im2);

    cv::Mat temp_im;
    cv::warpAffine(
      warped_im1,
      temp_im,
      T_rigid,
      cv::Size(warped_im1.cols, warped_im1.rows));

    cv::Mat sub = warped_im2 - temp_im;

    cv::namedWindow("Warped Subtraction");
    cv::imshow("Warped Subtraction", sub);

    // Now compare the result to the unwarped, transformed result:

    cv::warpAffine(
      im1_,
      temp_im,
      T_rigid,
      cv::Size(im1_.cols, im1_.rows));

    cv::Mat sub2 = im2_ - temp_im;
    cv::namedWindow("Subtraction");
    cv::imshow("Subtraction", sub2);

    cv::namedWindow("im2_");
    cv::imshow("im2_", im2_);
    cv::waitKey(0);
  }

  return R;
}
}  // namespace swri_image_util
