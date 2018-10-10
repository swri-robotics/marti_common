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

#ifndef OPENCV_UTIL_MODEL_FIT_H_
#define OPENCV_UTIL_MODEL_FIT_H_

#include <vector>

#include <opencv2/core/core.hpp>

#include <swri_math_util/random.h>
#include <swri_math_util/ransac.h>

#include <swri_opencv_util/models.h>

namespace swri_opencv_util
{
  template <class Model>
  cv::Mat FindModel2d(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr())
  {
    cv::Mat model;

    // Put data into the expected format.
    cv::Mat correspondences;
    if (!ZipCorrespondences(points1, points2, correspondences))
    {
      return model;
    }

    // Run RANSAC to robustly fit a rigid transform model to the set of
    // corresponding points.
    swri_math_util::Ransac<Model> ransac(rng);

    Model fit_model(correspondences);
    model = ransac.FitModel(
      fit_model, max_error, confidence, 1, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    // Populate output data.
    bool row_order = points1.rows > 1;
    if (row_order)
    {
      inliers1 = cv::Mat(good_points.size(), 1, CV_32FC2);
      inliers2 = cv::Mat(good_points.size(), 1, CV_32FC2);
      for (size_t i = 0; i < good_points.size(); ++i)
      {
        inliers1.at<cv::Vec2f>(i, 0) = points1.at<cv::Vec2f>(good_points[i], 0);
        inliers2.at<cv::Vec2f>(i, 0) = points2.at<cv::Vec2f>(good_points[i], 0);
      }
    }
    else
    {
      inliers1 = cv::Mat(1, good_points.size(), CV_32FC2);
      inliers2 = cv::Mat(1, good_points.size(), CV_32FC2);
      for (size_t i = 0; i < good_points.size(); ++i)
      {
        inliers1.at<cv::Vec2f>(0, i) = points1.at<cv::Vec2f>(0, good_points[i]);
        inliers2.at<cv::Vec2f>(0, i) = points2.at<cv::Vec2f>(0, good_points[i]);
      }
    }

    return model;
  }

  cv::Mat FindTranslation2d(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  cv::Mat FindRigidTransform2d(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  // Returns a 2x3 transform that can be applied to points1 to align them to
  // points2.
  cv::Mat FitRigidTransform2d(const cv::Mat& points1, const cv::Mat& points2);

  // Calculate 3x3 rotation matrix that can be applied to points1 to align them
  // to points2.
  cv::Mat FitRotation3d(const cv::Mat& points1, const cv::Mat& points2);

  cv::Mat FindAffineTransform2d(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  cv::Mat FitAffineTransform2d(const cv::Mat& points1, const cv::Mat& points2);

  cv::Mat FindHomography(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  PlaneModel FindPerpendicularPlaneWithPoint(
    const cv::Vec3f& point_on_plane,
    const cv::Vec3f& perp_axis,
    double max_angle_from_perp,
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t min_iterations,
    int32_t max_iterations,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  PlaneModel FindPlane(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t min_iterations = 1,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  PlaneModel FitPlane(const cv::Mat& points);

  LineModel3d FindLine3d(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t min_iterations = 1,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  LineModel3d FindOrthoLine3d(
    const cv::Mat& points,
    const LineModel3d& ortho,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t min_iterations = 1,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());

  LineModel3d FitLine3d(const cv::Mat& points);

  CrossModel3d FindCross3d(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t min_iterations = 1,
    int32_t max_iterations = 1000,
    swri_math_util::RandomGeneratorPtr rng = swri_math_util::RandomGeneratorPtr());
}

#endif  // OPENCV_UTIL_MODEL_FIT_H_
