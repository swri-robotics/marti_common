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

#include <swri_image_util/image_warp_util.h>
#include <swri_opencv_util/model_fit.h>

#include <algorithm>
#include <chrono>

#include <rclcpp/logging.hpp>

namespace swri_image_util
{
  cv::Mat WarpImage(const cv::Mat& image, double roll, double pitch)
  {
    cv::Mat warped;

    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    K.at<float>(0, 2) = static_cast<double>(image.cols - 1) / 2.0;
    K.at<float>(1, 2) = static_cast<double>(image.rows - 1) / 2.0;

    cv::Mat T = cv::Mat::zeros(cv::Size(3, 1), CV_32F);

    cv::Mat R = GetR(pitch, roll);

    cv::detail::PlaneWarper warper;
    warper.warp(image, K, R, T, cv::INTER_LANCZOS4, 0, warped);

    // TODO(malban): This warp can cause problems because it can change the
    //               image size.  The result should be cropped or padded.  The
    //               warp points function will need to be modified accordingly.

    return warped;
  }

  void WarpPoints(
      double pitch,
      double roll,
      const cv::Size& image_size,
      const cv::Mat& pts_in,
      cv::Mat& pts_out)
  {
    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    K.at<float>(0, 2) = static_cast<double>(image_size.width - 1) / 2.0;
    K.at<float>(1, 2) = static_cast<double>(image_size.height - 1) / 2.0;

    cv::detail::PlaneWarper warper;

    cv::Mat T = cv::Mat::zeros(cv::Size(3, 1), CV_32F);
    cv::Mat R = GetR(pitch, roll);
    pts_in.copyTo(pts_out);
    for (int32_t i = 0; i < pts_in.rows; ++i)
    {
      cv::Point2f pt;
      pt.x = pts_in.at<cv::Vec2f>(i, 0).val[0];
      pt.y = pts_in.at<cv::Vec2f>(i, 0).val[1];
      cv::Point2f pt2 = warper.warpPoint(pt, K, R, T);
      pts_out.at<cv::Vec2f>(i, 0).val[0] = pt2.x + K.at<float>(0, 2);
      pts_out.at<cv::Vec2f>(i, 0).val[1] = pt2.y + K.at<float>(1, 2);
    }
  }

  void WarpPoints(
      double pitch,
      double roll,
      const cv::Size& image_size,
      const std::vector<cv::KeyPoint>& pts_in,
      std::vector<cv::KeyPoint>& pts_out)
  {
    pts_out = pts_in;

    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    K.at<float>(0, 2) = static_cast<double>(image_size.width - 1) / 2.0;
    K.at<float>(1, 2) = static_cast<double>(image_size.height - 1) / 2.0;

    cv::detail::PlaneWarper warper;

    cv::Mat T = cv::Mat::zeros(cv::Size(3, 1), CV_32F);
    cv::Mat R = GetR(pitch, roll);

    for (uint32_t i = 0; i < pts_in.size(); i++)
    {
      pts_out[i].pt = warper.warpPoint(pts_in[i].pt, K, R, T);
      pts_out[i].pt.x += K.at<float>(0, 2);
      pts_out[i].pt.y += K.at<float>(1, 2);
    }
  }


  cv::Mat GetR(double pitch, double roll, double yaw)
  {
    cv::Mat R1 = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    cv::Mat R2 = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    cv::Mat R3 = cv::Mat::eye(cv::Size(3, 3), CV_32F);

    // do pitch first:
    R1.at<float>(0, 0) = std::cos(pitch);
    R1.at<float>(0, 2) = -std::sin(pitch);
    R1.at<float>(2, 0) = std::sin(pitch);
    R1.at<float>(2, 2) = std::cos(pitch);

    // Then roll
    R2.at<float>(1, 1) = std::cos(roll);
    R2.at<float>(1, 2) = std::sin(roll);
    R2.at<float>(2, 1) = -std::sin(roll);
    R2.at<float>(2, 2) = std::cos(roll);

    // Finally yaw
    R3.at<float>(0, 0) = std::cos(yaw);
    R3.at<float>(0, 1) = std::sin(yaw);
    R3.at<float>(1, 0) = -std::sin(yaw);
    R3.at<float>(1, 1) = std::cos(yaw);

    cv::Mat R = R3 * R2 * R1;

    return R;
  }

  cv::Mat PitchAndRollEstimator::EstimateNominalAngle(
      double& nominal_pitch,
      double& nominal_roll,
      bool show_image_diff,
      rclcpp::Logger logger)
  {
    if (kp1_matched_.empty() || kp2_matched_.empty())
    {
      return cv::Mat();
    }

    std::chrono::system_clock::time_point T1 = std::chrono::system_clock::now();
    cv::Mat T_rigid = EstimateNominalAngle(kp1_matched_,
                                           kp2_matched_,
                                           cv::Size(im1_.cols, im1_.rows),
                                           nominal_pitch,
                                           nominal_roll);

    std::chrono::system_clock::time_point T2 = std::chrono::system_clock::now();

    RCLCPP_ERROR(logger, "Estimate Nominal Angle time = %g",
        std::chrono::duration_cast<std::chrono::duration<float> >(T2 - T1).count());
    cv::Mat R = GetR(nominal_pitch, nominal_roll);

    if (show_image_diff)
    {
      // Do the warping and transformation and show the results
      cv::Mat warped_im1;
      cv::Mat warped_im2;

      warper_.warp(im1_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im1);
      warper_.warp(im2_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im2);

      cv::Mat temp_im;
      cv::warpAffine(warped_im1,
                     temp_im,
                     T_rigid,
                     cv::Size(warped_im1.cols, warped_im1.rows));

      cv::Mat sub = warped_im2 - temp_im;

      cv::namedWindow("Warped Subtraction");
      cv::imshow("Warped Subtraction", sub);

      // Now compare the result to the unwarped, transformed result:

      cv::warpAffine(im1_,
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

  cv::Mat PitchAndRollEstimator::EstimateNominalAngle(
      const cv::Mat& points1,
      const cv::Mat& points2,
      const cv::Size& image_size,
      double& nominal_pitch,
      double& nominal_roll)
  {
    // Max number of iterations per angle, per scale
    const int32_t max_iterations = 5;
    const int32_t num_octaves = 5;

    double pitch_range = 0.02 * 3.14159/180.0;
    double min_pitch = -std::abs(pitch_range / 2.0);
    double max_pitch = std::abs(pitch_range / 2.0);

    double roll_range = 0.02*3.14159/180.0;
    double min_roll = -std::abs(roll_range / 2.0);
    double max_roll = std::abs(roll_range / 2.0);

    cv::Mat T_rigid_final = cv::Mat();
    for (int32_t octave_idx = 0; octave_idx < num_octaves; ++octave_idx)
    {
      double dp = (max_pitch - min_pitch) /
          static_cast<double>(max_iterations - 1);
      double dr = (max_roll - min_roll) /
          static_cast<double>(max_iterations - 1);

      double min_diff = 1e20;
      nominal_pitch = 0.0;
      nominal_roll = 0.0;


      for (int32_t pitch_idx = 0; pitch_idx < max_iterations; ++pitch_idx)
      {
        double cur_pitch = min_pitch + dp * pitch_idx;
        for (int32_t roll_idx = 0; roll_idx < max_iterations; ++roll_idx)
        {
          double cur_roll = min_roll + dr * roll_idx;

          cv::Mat kp1_warped;
          swri_image_util::WarpPoints(cur_pitch,
                     cur_roll,
                     image_size,
                     points1,
                     kp1_warped);

          cv::Mat kp2_warped;
          swri_image_util::WarpPoints(cur_pitch,
                     cur_roll,
                     image_size,
                     points2,
                     kp2_warped);

          cv::Mat T_rigid;
          cv::Mat T_affine;
          double rms_error;
          bool success = EstimateTransforms(kp1_warped,
                                            kp2_warped,
                                            T_affine,
                                            T_rigid,
                                            rms_error);
          if (!success)
          {
            continue;
          }

          double cur_diff = rms_error;

          if (cur_diff < min_diff)
          {
            min_diff = cur_diff;

            nominal_pitch = cur_pitch;
            nominal_roll = cur_roll;
            T_rigid_final = T_rigid;
          }
        }
      }

      min_pitch = nominal_pitch - std::abs(dp * 2 / 3);
      max_pitch = nominal_pitch + std::abs(dp * 2 / 3);


      min_roll = nominal_roll - std::abs(dr * 2 / 3);
      max_roll = nominal_roll + std::abs(dr * 2 / 3);
    }

    cv::Mat R = GetR(nominal_pitch,
                     nominal_roll);

    return T_rigid_final;
  }

  bool PitchAndRollEstimator::ComputeGeometricMatches(rclcpp::Logger logger)
  {
    if (im1_.empty() || im2_.empty())
    {
      RCLCPP_ERROR(logger, "No images defined");
      return false;
    }

    // perform the matching first:
    // Compute the matching features between this frame and the previous one.
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(descriptors1_, descriptors2_, matches);

    cv::Mat points1;
    cv::Mat points2;
    ConvertMatches(kp1_, kp2_, matches, points1, points2);

    // Compute the fundamental matrix which describes the camera motion
    // between the frames using a RANSAC process and get the set of inlier
    // matches which agree with the motion.
    cv::Mat fundamental_matrix;
    cv::Mat fund_inliers1;
    cv::Mat fund_inliers2;
    try
    {
      GetFundamentalInliers(points1,
                            points2,
                            fundamental_matrix,
                            fund_inliers1,
                            fund_inliers2);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Caught an exception when computing fundamental inliers:"
                " %s", e.what());
      return false;
    }

    RCLCPP_INFO(logger, "Found %d fundamental inliers.", fund_inliers1.rows);

    cv::Mat inliers1;
    cv::Mat inliers2;
    std::vector<uint32_t> good_points;
    
    int32_t iterations;
    cv::Mat affine = swri_opencv_util::FindAffineTransform2d(
      fund_inliers1, fund_inliers2, inliers1, inliers2, good_points, iterations, 30.0);

    if (affine.empty())
    {
      RCLCPP_ERROR(logger, "Failed to compute 2D affine transform.");
      return false;
    }

    kp1_matched_ = inliers1;
    kp2_matched_ = inliers2;

    return true;
  }

  bool PitchAndRollEstimator::EstimateTransforms(
      cv::Mat& pts1,
      cv::Mat& pts2,
      cv::Mat& T_affine,
      cv::Mat& T_rigid,
      double& rms_error)
  {
    cv::Mat inliers1;
    cv::Mat inliers2;
    std::vector<uint32_t> good_points;
    
    int32_t iterations;
    T_affine = swri_opencv_util::FindAffineTransform2d(
      pts1, pts2, inliers1, inliers2, good_points, iterations, 30.0);
    
    T_rigid = swri_opencv_util::FindRigidTransform2d(
      pts1, pts2, inliers1, inliers2, good_points, iterations, 30.0);
    
    cv::Mat inliers1_t;
    cv::transform(inliers1, inliers1_t, T_rigid);
    double n = good_points.size();
    rms_error = cv::norm(inliers2, inliers1_t, cv::NORM_L2) / std::sqrt(n);

    if (T_rigid.empty())
    {
      return false;
    }

    return true;
  }

  void PitchAndRollEstimator::WarpPoints(
      double pitch,
      double roll,
      const cv::Mat& pts_in,
      cv::Mat& pts_out,
      rclcpp::Logger logger)
  {
    if (im1_.empty() || im2_.empty())
    {
      RCLCPP_ERROR(logger, "Object not initialized. Pitch and roll not computed.  Perhaps"
                "call static implementation instead");
      return;
    }

    swri_image_util::WarpPoints(pitch,
               roll,
               cv::Size(im1_.cols, im1_.rows),
               pts_in,
               pts_out);
  }

  void PitchAndRollEstimator::WarpAffinePoints(
      const cv::Mat& T,
      const cv::Mat& pts_in,
      cv::Mat& pts_out)
  {
    // Create augmented keypoint matrix:
    cv::Mat aug_mat(cv::Size(3, pts_in.rows), CV_32F);

    for (int32_t i = 0; i < pts_in.rows; ++i)
    {
      aug_mat.at<float>(i, 0) = pts_in.at<cv::Vec2f>(0, i)[0];
      aug_mat.at<float>(i, 1) = pts_in.at<cv::Vec2f>(0, i)[1];
      aug_mat.at<float>(i, 2) = 1.0;
    }

    cv::Mat T_temp = T.t();
    cv::Mat pts_out_temp = aug_mat * T_temp;

    pts_out.release();
    pts_out.create(cv::Size(1, pts_in.rows), CV_32FC2);
    // Convert points back to proper form:
    for (int32_t i = 0; i < pts_in.rows; ++i)
    {
      pts_out.at<cv::Vec2f>(0, i)[0] = pts_out_temp.at<float>(i, 0);
      pts_out.at<cv::Vec2f>(0, i)[1] = pts_out_temp.at<float>(i, 1);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////   PitchAndRollEstimatorQueue   ////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  //
  // PitchAndRollEstimatorQueue()
  //
  //////////////////////////////////////////////////////////////////////////////
  PitchAndRollEstimatorQueue::PitchAndRollEstimatorQueue() :
    filter_idx_(0),
    num_elements_(0)
  {
    SetBufferSize();
    ComputeStats();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // SetBufferSize()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::SetBufferSize(size_t buff_size)
  {
    pitches_.reserve(buff_size);
    rolls_.reserve(buff_size);
    filter_idx_ = std::min(filter_idx_, buff_size - 1);
    num_elements_ = std::min(num_elements_, buff_size);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // WarpPoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::WarpPoints(const cv::Mat& points_in,
                                              cv::Mat& points_out,
                                              const cv::Size& image_size,
                                              bool use_median)
  {
    double pitch;
    double roll;
    if (use_median)
    {
      GetMedianPitchAndRoll(pitch, roll);
    }
    else
    {
      GetMeanPitchAndRoll(pitch, roll);
    }

    swri_image_util::WarpPoints(pitch, roll, image_size, points_in, points_out);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // Clear()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::Clear()
  {
    pitches_.clear();
    rolls_.clear();
    filter_idx_ = 0;
    num_elements_ = 0;
    ComputeStats();
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // GenerateNewEstimate()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::GenerateNewEstimate(
      const cv::Mat& points1,
      const cv::Mat& points2,
      const cv::Size& image_size)
  {
    double pitch = 0.0;
    double roll = 0.0;
    cv::Mat T = PitchAndRollEstimator::EstimateNominalAngle(points1,
                                                            points2,
                                                            image_size,
                                                            pitch,
                                                            roll);

    if (!T.empty())
    {
      LoadNewData(pitch, roll);
      ComputeStats();
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // GetMeanPitchAndRoll()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimatorQueue::GetMeanPitchAndRoll(double& pitch,
                                                       double& roll)
  {
    pitch = mean_pitch_;
    roll = mean_roll_;

    return num_elements_ > 0;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // GetMedianPitchAndRoll()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimatorQueue::GetMedianPitchAndRoll(double& pitch,
                                                         double& roll)
  {
    pitch = median_pitch_;
    roll = median_roll_;

    return num_elements_ > 0;
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // LoadNewData()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::LoadNewData(double new_pitch,
                                               double new_roll)
  {
    pitches_[filter_idx_] = new_pitch;
    rolls_[filter_idx_] = new_roll;
    filter_idx_ = (filter_idx_ + 1) % pitches_.size();
    num_elements_ += 1;
    num_elements_ = std::min(num_elements_, pitches_.size());

    ComputeStats();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ComputeStats()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::ComputeStats()
  {
    mean_pitch_ = 0.0;
    mean_roll_ = 0.0;
    median_pitch_ = 0.0;
    median_roll_ = 0.0;

    if (num_elements_ == 0)
    {
      return;
    }

    std::vector<double> temp_pitch(pitches_.begin(), pitches_.begin() + (num_elements_ - 1));
    std::vector<double> temp_roll(rolls_.begin(), rolls_.begin() + (num_elements_ - 1));

    std::sort(temp_pitch.begin(), temp_pitch.end());
    std::sort(temp_roll.begin(), temp_roll.end());

    double pitch_sum = 0.0;
    double roll_sum = 0.0;

    for (int32_t i = 0; i < (int32_t)temp_pitch.size(); ++i)
    {
      pitch_sum += temp_pitch[i];
      roll_sum += temp_roll[i];
    }

    double N = static_cast<double>(temp_pitch.size());
    mean_pitch_ = pitch_sum / N;
    mean_roll_ = roll_sum / N;

    int32_t mid_idx = static_cast<int32_t>(temp_pitch.size() - 1) / 2;
    if (temp_pitch.size() % 2 == 0)
    {
      median_pitch_ = (temp_pitch[mid_idx] + temp_pitch[mid_idx + 1]) / 2.0;
      median_roll_ = (temp_roll[mid_idx] + temp_roll[mid_idx + 1]) / 2.0;
    }
    else
    {
      median_pitch_ = temp_pitch[mid_idx];
      median_roll_ = temp_roll[mid_idx];
    }
  }
}
