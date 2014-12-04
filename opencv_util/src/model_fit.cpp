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
//     * Neither the name of the <organization> nor the
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

#include <opencv_util/models.h>
#include <math_util/ransac.h>

namespace opencv_util
{
  cv::Mat FindTranslation2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    cv::Mat model;
    
    // Put data into the expected format.
    std::vector<cv::Vec4f> matched_points;
    if (!ConvertToVec4f(points1, points2, matched_points))
    {
      return model;
    }
    
    // Run RANSAC to robustly fit a rigid transform model to the set of 
    // corresponding points.
    math_util::Ransac<Translation2d> ransac(rng);
    model = ransac.FitModel(
      matched_points, max_error, confidence, max_iterations, good_points);
    
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
    
    // Calculate the refined transform using least squares on the inlier points.
    //model = FitTranslation2d(inliers1, inliers2);

    return model;
  }

  cv::Mat FindRigidTransform2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    cv::Mat model;
    
    // Put data into the expected format.
    std::vector<cv::Vec4f> matched_points;
    if (!ConvertToVec4f(points1, points2, matched_points))
    {
      return model;
    }
    
    // Run RANSAC to robustly fit a rigid transform model to the set of 
    // corresponding points.
    math_util::Ransac<RigidTransform2d> ransac(rng);
    model = ransac.FitModel(
      matched_points, max_error, confidence, max_iterations, good_points);
    
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
    
    // Calculate the refined transform using least squares on the inlier points.
    //model = FitRigidTransform2d(inliers1, inliers2);

    return model;
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
    double max_error,
    double confidence,
    int32_t max_iterations,
    math_util::RandomGeneratorPtr rng)
  {
    cv::Mat model;
    
    // Put data into the expected format.
    std::vector<cv::Vec4f> matched_points;
    if (!ConvertToVec4f(points1, points2, matched_points))
    {
      return model;
    }
    
    // Run RANSAC to robustly fit an affine transform model to the set of 
    // corresponding points.
    math_util::Ransac<AffineTransform2d> ransac(rng);
    model = ransac.FitModel(
      matched_points, max_error, confidence, max_iterations, good_points);
    
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
    
    // Calculate the refined transform using least squares on the inlier points.
    //model = FitAffineTransform2d(inliers1, inliers2);

    return model;
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
