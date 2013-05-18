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

#ifndef IMAGE_UTIL_MOTION_ESTIMATION_H_
#define IMAGE_UTIL_MOTION_ESTIMATION_H_

#include <vector>

// ROS Libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// OpenCV LIbraries
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>


#ifndef LA_COMPLEX_SUPPORT
#define LA_COMPLEX_SUPPORT
#endif

#include <lapackpp.h>
#include <gfqrc.h>

namespace image_util
{
  cv::Mat LeastSqauresRigid2DTransform(
    const cv::Mat& inliers1,
    const cv::Mat& inliers2);

  bool ValidPointsForTransform(
    const cv::Mat& points1,
    const cv::Mat& points2);

  double CalculateReprojectionError(
    const cv::Mat& points1,
    const cv::Mat& points2,
    const cv::Mat& transform);

  tf::Transform ToTransform(const cv::Mat& matrix);

  /**
   * @brief Prints out the matrix to the console
   *
   * @param[in]  A  The matrix to print
   */
  void PrintMatrix(const LaGenMatDouble &A);

  /**
   * @brief This function extracts the Rotation and Translation (up to a scale
   *        factor) embedded in the fundamental matrix
   *
   * The fundamental matrix represents the constraint between two uncalibrated
   * camera views of the same scene.  A common point in 3D space as viewed by
   * both cameras (or two views from the same camera) satisfies
   * (x1' * F * x2 = 0). Note that from a single camera the magnitude of the
   * translation (in the 3D coordinate system) is undetermined.
   *
   *
   * @param[in]  F              The fundamental matrix
   * @param[in]  Intrinsics     The camera matrix representing the intrinsic
   *                            parameters (i.e. the pinhole camera model)
   * @param[out] R1             One of the computed rotation matrices (one of
   *                            two possible solutions)
   * @param[out] R2             The second of the two possible rotation matrices
   * @param[out] T              The computed translation vector (which should be
   *                            accurate in direction, but not in magnitude)
   *
   */
  void ExtractMotionParameters(
      const cv::Mat& Fin,
      const cv::Mat& Intrinsics,
      cv::Mat& R1,
      cv::Mat& R2,
      cv::Mat& T);

  /**
   * @brief Computes the rigid planar transformation given the points passed in
   *
   * Returns an empty Mat (use empty() to check) if no valid transform is found.
   * Note that the Transformation is in transposed abbreviated form
   * (i.e. X1'*T_abb' = X2'), where the [0,0,1] row is left off
   *
   * @param[in]  points1  The source points (points in the first frame) -- This
   *                      should be a 2 channel float32 (CV_32FC2) cv::Mat with
   *                      the first channel corresponding to the x values and
   *                      the second channel corresponding to the y values
   * @param[in]  points2  The destination points (points in the second frame)
   *                      -- This should be a 2 channel float32 (CV_32FC2)
   *                      cv::Mat with the first channel corresponding to the x
   *                      values and the second channel corresponding to the y
   *                      values
   * @param[out] inliers1 The inlier source points which support the transform.
   * @param[out] inliers2 The inlier destination points which suppor the
   *                      transform.
   * @param[in]  good_points     This is just to maintain similarity to the
   *                             estimateRigidTransform from openCV.
   *
   * @retval  Returns the transformation matrix, which will be empty if no valid
   *          transformation was found
   */
  cv::Mat ComputeRigid2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2,
      cv::Mat& inliers1,
      cv::Mat& inliers2,
      std::vector<uint32_t> &good_points);

  /**
   * @brief Computes the rigid planar transformation given the points passed in
   *
   * Returns an empty Mat (use empty() to check) if no valid transform is found.
   * Note that the Transformation is in transposed abbreviated form
   * (i.e. X1'*T_abb' = X2'), where the [0,0,1] row is left off
   *
   * @param[in]  points1  The source points (points in the first frame) -- This
   *                      should be a 2 channel float32 (CV_32FC2) cv::Mat with
   *                      the first channel corresponding to the x values and
   *                      the second channel corresponding to the y values
   * @param[in]  points2  The destination points (points in the second frame) --
   *                      This should be a 2 channel float32 (CV_32FC2) cv::Mat
   *                      with the first channel corresponding to the x values
   *                      and the second channel corresponding to the y values
   * @param[in] temp
   *
   * @retval  Returns the transformation matrix, which will be empty if no valid
   *          transformation was found
   */
  inline cv::Mat ComputeRigid2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2)
  {
      cv::Mat inliers1;
      cv::Mat inliers2;
      std::vector<uint32_t> good_points;
      return ComputeRigid2DTransform(
          points1, points2, inliers1, inliers2, good_points);
  }


  /**
   * @brief Computes the rigid planar transformation given the points passed in
   *
   * Returns an empty Mat (use empty() to check) if no valid transform is found.
   * Note that the Transformation is in transposed abbreviated form
   * (i.e. X1'*T_abb' = X2'), where the [0,0,1] row is left off
   *
   * @param[in]  points1  The source points (points in the first frame) -- This
   *                      should be a 2 channel float32 (CV_32FC2) cv::Mat with
   *                      the first channel corresponding to the x values and
   *                      the second channel corresponding to the y values
   * @param[in]  points2  The destination points (points in the second frame) --
   *                      This should be a 2 channel float32 (CV_32FC2) cv::Mat
   *                      with the first channel corresponding to the x values
   *                      and the second channel corresponding to the y values
   * @param[out] inliers1
   * @param[out] inliers2
   *
   * @retval  Returns the transformation matrix, which will be empty if no valid
   *          transformation was found
   */
  inline cv::Mat ComputeRigid2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2,
      cv::Mat& inliers1,
      cv::Mat& inliers2)
  {
      std::vector<uint32_t> good_points;
      return ComputeRigid2DTransform(
          points1, points2, inliers1, inliers2, good_points);
  }

  /**
   * @brief      Computes an Affine Transformation for
   *
   * @param[in]  points1
   * @param[in]  points2
   * @param[out] inliers1
   * @param[out] inliers2
   * @param[out] T_rigid
   * @param[out] rms_error    The RMS (distance) error for inlier points using
   *                          the rigid transform
   *
   * @return
   */
  cv::Mat ComputeLooseAffine2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2,
      cv::Mat& inliers1,
      cv::Mat& inliers2,
      cv::Mat& T_rigid,
      double& rms_error);


  /**
   * @brief Computes the rigid 3D (6DOF) transformation given the points passed
   *        in
   *
   * Returns an empty Mat (use empty() to check) if no valid transform is found.
   * Note that the Transformation is in transposed abbreviated form
   * (i.e. X1'*T_abb' = X2'), where the [0,0,0,1] row is left off
   *
   * @param[in]  points1  The source points (points in the first frame) -- This
   *                      should be a 3 channel float32 (CV_32FC3) cv::Mat with
   *                      the first channel corresponding to the x values and
   *                      the second channel corresponding to the y values and
   *                      the third channel corresponding to the z values
   * @param[in]  points2  The destination points (points in the second frame) --
   *                      This should be a 3 channel float32 (CV_32FC3) cv::Mat
   *                      with the first channel corresponding to the x values
   *                      and the second channel corresponding to the y values
   *                      and the third channel corresponding to the z values
   *
   * @param[in]  good_points points within bound of the transform computed
   */
  cv::Mat ComputeRigid3DTransform(cv::Mat& points1, cv::Mat& points2);

  /**
   * @brief  Converts the rotation matrix portion of a non-rigid transform
   *         matrix to one with the the "nearest" orthonormal rotation matrix
   *
   * Uses SVD to create the nearest orthonormal basis.
   *
   * @param[in/out]  T             Non-rigid transform in / Rigid Transform out
   * @param[out]     conditionNum  Returns the condition number of the original
   *                               rot matrix
   * @param[out]     rnorm         Returns ||RtR - StS|| where S is diagonal of
   *                               the singular value matrix
   */
  void RegularizeTransform(
      LaGenMatDouble &T,
      double &conditionNum,
      double &rnorm);

  /**
   * @brief  Converts a non-orthonormal matrix to the "nearest" orthonormal
   *         matrix
   *
   * Uses SVD to create the nearest orthonormal basis.
   *
   * @param[in/out]  rot           Approximate rotation matrix in,
   *                               orthonormalized rotation matrix out
   * @param[out]     conditionNum  Returns the condition number of the original
   *                               rot matrix
   * @param[out]     rnorm         Returns ||RtR - StS|| where S is diagonal of
   *                               the singular value matrix
   */
  void RegularizeRotation(
      LaGenMatDouble &rot,
      double &conditionNum,
      double &rnorm);

  /**
   * @brief  Generates a set of non-repeating indices from set of ordered
   *         integers
   *
   * This function generates a random permutation consisting of a subset
   * (total_samples) with non-repeating elements of ordered integers
   * (1,2,3,...N)
   *
   * @param[in]   max_num               Maximum number from which to draw
   *                                    samples
   * @param[in]   total_samples         Total number of samples to draw
   * @param[out]  indices               A vector of indices
   */
  void RandPermSet(
      uint32_t max_num,
      uint32_t total_samples,
      std::vector<uint32_t>& indices);

  /**
   * @brief  Extracts the rotation matrix from a transformation matrix
   *
   * @param[in]  T  Transformation matrix
   * @param[out] R  Rotation matrix
   */
  void GetR(const LaGenMatDouble& T, LaGenMatDouble& R);
}

#endif  // IMAGE_UTIL_MOTION_ESTIMATION_H_
