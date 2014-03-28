// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <opencv_util/model_fit.h>

#include <opencv_util/models.h>
#include <math_util/ransac.h>

namespace opencv_util
{
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
    
    // Perform least squares fit on inliers to refine model.
    //    For least squares there are several decomposition methods:
    //       DECOMP_LU
    //       DECOMP_CHOLESKY ([A] must be symmetrical)
    //       DECOMP_EIG ([A] must be symmetrical)
    //       DECOMP_SVD
    //       DECOMP_QR
    cv::Mat A(good_points.size(), 3, CV_32F);
    cv::Mat B = inliers2.reshape(1, 2);
    if (row_order)
    {
      for (size_t i = 0; i < good_points.size(); ++i)
      {
        const cv::Vec2f& point = inliers1.at<cv::Vec2f>(i, 0);
        cv::Vec3f& A_i = A.at<cv::Vec3f>(i, 0);
        A_i[0] = point[0];
        A_i[1] = point[1];
        A_i[2] = 1.0;
      }
    }
    else
    {
      B = inliers2.t();
      B = B.reshape(1, 2);
      
      for (size_t i = 0; i < good_points.size(); ++i)
      {
        const cv::Vec2f& point = inliers1.at<cv::Vec2f>(0, i);
        cv::Vec3f& A_i = A.at<cv::Vec3f>(i, 0);
        A_i[0] = point[0];
        A_i[1] = point[1];
        A_i[2] = 1.0;
      }
    }
    
    cv::Mat x;
    if (cv::solve(A, B, x))
    {
      model = x;
    }

    return model;
  }
}
