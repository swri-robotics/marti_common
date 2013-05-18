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
/*
 * image_warp_util.h
 *
 *  Created on: Jul 25, 2012
 *      Author: kkozak
 */

#ifndef IMAGE_UTIL_IMAGE_WARP_UTIL_H_
#define IMAGE_UTIL_IMAGE_WARP_UTIL_H_

#include <vector>

// Boost Libraries
#include <boost/circular_buffer.hpp>

// ROS Libraries
#include <ros/ros.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/stitching/detail/warpers.hpp>

#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4
#include <opencv2/nonfree/features2d.hpp>
#endif

// RANGER Libraries
#include <image_util/motion_estimation.h>
#include <image_util/image_matching.h>

namespace image_util
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
     * @brief      Constructor
     *
     * @param[in]  im1      The first image
     * @param[in]  im2      The second image
     */
    PitchAndRollEstimator(const cv::Mat& im1,
                          const cv::Mat& im2);


    /**
     * @brief      Loads two images into the object.  This is a required first
     *             step in order to use the other functions in the class
     *
     * @param[in]  im1      An image of a roughly planar surface
     * @param[in]  im2      An image that partially overlaps the first image
     *
     * @retval     Returns false if unsuccessful anywhere in the process of
     *             initializing the images, keypoints and matches.
     */
    bool LoadImages(const cv::Mat& im1,
                    const cv::Mat& im2);


    /**
     * @brief      Estimates the nominal pitch and roll of the camera (from
     *             perfectly vertical) from two overlapping images.
     *
     * @return     Returns the rotation matrix for the computed pitch and roll
     */
    cv::Mat EstimateNominalAngle(double& nominal_pitch,
                                 double& nominal_roll,
                                 bool show_image_diff = false);


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
     * @brief      Detects features in the specified image using a SURF feature
     *             detector (the hessian threshold will be adjusted adaptively
     *             to detect a number of features within the specified min-max
     *             range.
     *
     * @param[in]  image          The image in which to detect features
     * @param[out] keypoints      The detected keypoints
     * @param[out] descriptors    The descriptors extracted for each keypoint
     * @param[in]  min_keypoints  The minimum number of keypoints to find
     * @param[in]  max_keypoints  The maximum number to find
     *
     * @retval     Returns false if unable to detect keypoints, or number of
     *             keypoints detected falls outside of the specified range
     */
    static bool GetKeypoints(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints,
                             cv::Mat& descriptors,
                             int32_t min_keypoints = 500,
                             int32_t max_keypoints = 800);

    /**
     * @brief      Matches keypoints using loose geometric constraints and
     *             stores them in kp1_matched_ and kp2_matched_
     *
     * @retval     Returns false if unable to find valid matches
     */
    bool ComputeGeometricMatches();


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
                    cv::Mat& pts_out);

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
     * @brief      Estimates pitch and roll from two overlapping images and
     *             loads the pitch and roll data onto the buffer
     *
     * @param[in]  im1      Points from first image
     * @param[in]  im2      Corresponding points from second image
     */
    void GenerateNewEstimate(const cv::Mat& im1,
                             const cv::Mat& im2);

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
