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

#ifndef IMAGE_UTIL_IMAGE_WARP_UTIL_H_
#define IMAGE_UTIL_IMAGE_WARP_UTIL_H_

#include <vector>

// Boost Libraries
#include <boost/circular_buffer.hpp>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching/detail/warpers.hpp>

// RANGER Libraries
#include <swri_image_util/image_matching.h>

#include <rclcpp/logger.hpp>

namespace swri_image_util
{
  cv::Mat WarpImage(const cv::Mat& image, double roll, double pitch);

  /**
   * Warps a matrix of points (in the same form as the inliers)
   *
   * @param[in]  pitch      The pitch used to warp the point
   * @param[in]  roll       The roll used to warp the point
   * @param[in]  image_size The size of the (unwarped) image
   * @param[in]  pts_in     The points to warp
   * @param[out] pts_out    The warped points
   */
  void WarpPoints(
      double pitch,
      double roll,
      const cv::Size& image_size,
      const cv::Mat& pts_in,
      cv::Mat& pts_out);

  /**
   * Warps a matrix of points (in the same form as the inliers)
   *
   * @param[in]  pitch      The pitch used to warp the point
   * @param[in]  roll       The roll used to warp the point
   * @param[in]  image_size The size of the (unwarped) image
   * @param[in]  pts_in     The points to warp
   * @param[out] pts_out    The warped points
   */
  void WarpPoints(
      double pitch,
      double roll,
      const cv::Size& image_size,
      const std::vector<cv::KeyPoint>& pts_in,
      std::vector<cv::KeyPoint>& pts_out);

  /**
   * @brief      Gets the rotation matrix associated with the specified pitch
   *             and roll values
   *
   * @param[in]  pitch    The pitch value
   * @param[in]  roll     The roll value
   * @param[in]  yaw      The yaw value (default = 0.0);
   *
   * @retval     Returns the appropriately formatted rotation matrix
   */
  cv::Mat GetR(double pitch, double roll, double yaw = 0.0);


  /**
   * @brief      A class for estimating image warping based on perspective
   *             distortion.  Primarily intended for use with downward-facing
   *             camera methods.
   */
  class PitchAndRollEstimator
  {
  public:
    /**
     * @brief      Constructor
     */
    PitchAndRollEstimator() {}


    /**
     * @brief      Estimates the nominal pitch and roll of the camera (from
     *             perfectly vertical) from two overlapping images.
     *
     * @return     Returns the rotation matrix for the computed pitch and roll
     */
    cv::Mat EstimateNominalAngle(double& nominal_pitch,
                                 double& nominal_roll,
                                 bool show_image_diff,
                                 rclcpp::Logger logger=rclcpp::get_logger("swri_image_util"));


    /**
     * @brief      Estimates the nominal pitch and roll of the camera (from
     *             perfectly vertical) from two overlapping images.
     *
     * @return     Returns the rotation matrix for the computed pitch and roll
     */
    static cv::Mat EstimateNominalAngle(const cv::Mat& points1,
                                        const cv::Mat& points2,
                                        const cv::Size& image_size,
                                        double& nominal_pitch,
                                        double& nominal_roll);

  private:
    cv::Mat im1_;
    cv::Mat im2_;

    cv::Mat K_;
    cv::Mat T_;

    std::vector<cv::KeyPoint> kp1_;
    std::vector<cv::KeyPoint> kp2_;
    cv::Mat descriptors1_;
    cv::Mat descriptors2_;

    cv::Mat kp1_matched_;
    cv::Mat kp2_matched_;

    cv::detail::PlaneWarper warper_;

    /**
     * @brief      Matches keypoints using loose geometric constraints and
     *             stores them in kp1_matched_ and kp2_matched_
     *
     * @retval     Returns false if unable to find valid matches
     */
    bool ComputeGeometricMatches(rclcpp::Logger logger=rclcpp::get_logger("swri_image_util"));


    /**
     * @brief      Estimates the "nearest" rigid, and corresponding full affine
     *             transformation for a set of matching points
     *
     * @param[in]  pts1
     * @param[in]  pts2
     * @param[out] T_affine
     * @param[out] T_rigid
     * @param[out] rms_error    The RMS (distance) error for inlier points using
     *                          the rigid transform
     *
     * @retval     Returns false if unable to find enough valid matches
     */
    static bool EstimateTransforms(cv::Mat& pts1,
                                   cv::Mat& pts2,
                                   cv::Mat& T_affine,
                                   cv::Mat& T_rigid,
                                   double& rms_error);

    /**
     * @brief      Warps a matrix of points (in the same form as the inliers)
     *
     * @param[in]  pitch      The pitch used to warp the point
     * @param[in]  roll       The roll used to warp the point
     * @param[in]  pts_in     The points to warp
     * @param[out] pts_out    The warped points
     */
    void WarpPoints(double pitch,
                    double roll,
                    const cv::Mat& pts_in,
                    cv::Mat& pts_out,
                    rclcpp::Logger logger=rclcpp::get_logger("swri_image_util"));

    /**
     * @brief      Warps a matrix of points (in the same form as the inliers)
     *
     * @param[in]  T          The 2D rigid transform to use for warping
     * @param[in]  pts_in     The points to warp
     * @param[out] pts_out    The warped points
     */
    void WarpAffinePoints(const cv::Mat& T,
                          const cv::Mat& pts_in,
                          cv::Mat& pts_out);
  };

  /**
   * @brief      A class for estimating image warping based on perspective
   *             distortion.  Primarily intended for use with downward-facing
   *             camera methods.
   */
  class PitchAndRollEstimatorQueue
  {
  public:
    /**
     * @brief      Constructor
     */
    PitchAndRollEstimatorQueue();

    ~PitchAndRollEstimatorQueue() {}

    /**
     * @brief      Sets the circular buffer capacity for computing statistics
     *
     * @param[in]  buff_size    The desired size of the buffer
     */
    void SetBufferSize(int32_t buff_size = 50);

    /**
     * @brief      Clears the buffer
     */
    void Clear();

    /**
     * @brief      Warps points based on the stored estimated pitch and roll
     *
     * @param[in]  points_in      The input points matrix
     * @param[out] points_out     The output points matrix
     * @param[in]  image_size     The corresponding image size
     * @param[in]  use_median     Specify whether to use median or mean values,
     *                            default is use_median = true
     */
    void WarpPoints(const cv::Mat& points_in,
                    cv::Mat& points_out,
                    const cv::Size& image_size,
                    bool use_median = true);

    /**
     * @brief      Estimates pitch and roll from corresponding points and loads
     *             the pitch and roll data onto the buffer
     *
     * @param[in]  points1      Points from first image
     * @param[in]  points2      Corresponding points from second image
     * @param[in]  image_size   The size of the image
     */
    void GenerateNewEstimate(const cv::Mat& points1,
                             const cv::Mat& points2,
                             const cv::Size& image_size);

    /**
     * @brief      Loads new pitch and roll data directly onto the buffer
     *
     * @param[in]  new_pitch    New pitch data
     * @param[in]  new_roll     New roll data
     */
    void LoadNewData(double new_pitch,
                     double new_roll);

    /**
     * @brief      Computes the mean pitch and roll
     *
     * @param[out] pitch    The computed mean pitch (will be zero if buffer is
     *                      empty)
     * @param[out] roll     The computed mean roll  (will be zero if buffer is
     *                      empty)
     *
     * @retval     Returns false if the buffer is empty
     */
    bool GetMeanPitchAndRoll(double& pitch,
                             double& roll);


    /**
     * @brief      Computes the median pitch and roll
     *
     * @param[out] pitch    The computed median pitch (will be zero if buffer is
     *                      empty)
     * @param[out] roll     The computed median roll (will be zero if buffer is
     *                      empty)
     *
     * @retval     Returns false if the buffer is empty
     */
    bool GetMedianPitchAndRoll(double& pitch,
                               double& roll);

  private:
    boost::circular_buffer<double> pitches_;
    boost::circular_buffer<double> rolls_;

    double mean_pitch_;
    double mean_roll_;
    double median_pitch_;
    double median_roll_;

    /**
     * @brief      Computes the statistics on the data in the buffers
     */
    void ComputeStats();
  };
}

#endif  // IMAGE_UTIL_IMAGE_WARP_UTIL_H_
