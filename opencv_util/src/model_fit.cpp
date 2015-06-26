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

#include <opencv_util/model_fit.h>

namespace opencv_util
{
  cv::Mat FindTranslation2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    return FindModel2d<Translation2d>(
      points1, points2, inliers1, inliers2, good_points, iterations, max_error,
      confidence, max_iterations, rng);
  }

  cv::Mat FindRigidTransform2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    return FindModel2d<RigidTransform2d>(
      points1, points2, inliers1, inliers2, good_points, iterations, max_error,
      confidence, max_iterations, rng);
  }
  
  cv::Mat FitRigidTransform2d(const cv::Mat& points1, const cv::Mat& points2)
  {
    cv::Mat transform;
    
    return transform;
  }
  
  cv::Mat FindAffineTransform2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    return FindModel2d<AffineTransform2d>(
      points1, points2, inliers1, inliers2, good_points, iterations, max_error,
      confidence, max_iterations, rng);
  }

  cv::Mat FindHomography(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    return FindModel2d<Homography>(
      points1, points2, inliers1, inliers2, good_points, iterations, max_error,
      confidence, max_iterations, rng);
  }

  cv::Mat FitAffineTransform2d(const cv::Mat& points1, const cv::Mat& points2)
  {
    cv::Mat transform;
    
    if (!Valid2dPointCorrespondences(points1, points2))
    {
      return transform;
    }
    
    bool row_order = points1.rows > 1;
    int32_t size = row_order ? points1.rows : points1.cols;
    
    // Perform least squares fit on inliers to refine model.
    //    For least squares there are several decomposition methods:
    //       DECOMP_LU
    //       DECOMP_CHOLESKY ([A] must be symmetrical)
    //       DECOMP_EIG ([A] must be symmetrical)
    //       DECOMP_SVD
    //       DECOMP_QR
    cv::Mat A(size, 3, CV_32F);
    cv::Mat B = points2.reshape(1, 2);
    if (row_order)
    {
      for (int32_t i = 0; i < size; ++i)
      {
        const cv::Vec2f& point = points1.at<cv::Vec2f>(i, 0);
        cv::Vec3f& A_i = A.at<cv::Vec3f>(i, 0);
        A_i[0] = point[0];
        A_i[1] = point[1];
        A_i[2] = 1.0;
      }
    }
    else
    {
      B = points2.t();
      B = B.reshape(1, 2);
      
      for (int32_t i = 0; i < size; ++i)
      {
        const cv::Vec2f& point = points1.at<cv::Vec2f>(0, i);
        cv::Vec3f& A_i = A.at<cv::Vec3f>(i, 0);
        A_i[0] = point[0];
        A_i[1] = point[1];
        A_i[2] = 1.0;
      }
    }
    
    cv::Mat x;
    if (cv::solve(A, B, x))
    {
      transform = x;
    }

    return transform;
  }
}
