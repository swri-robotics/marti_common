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

#include <opencv_util/models.h>

namespace opencv_util
{
  bool RigidTransform2d::GetModel(const std::vector<T>& data, M& model)
  {
    // TODO(malban)
    return false;
  }
  
  double RigidTransform2d::GetError(const T& data, const M& model)
  {
    // TODO(malban)
    return 0.0;
  }


  bool Valid2dPointCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2)
  {
    if (points1.type() != points2.type())
    {
      return false;
    }
    
    if (points1.type() != CV_32FC2)
    {
      return false;
    }
    
    if (points1.cols != points2.cols  || points1.rows  != points2.rows)
    {
      return false;
    }
    
    if (points1.cols != 1 && points1.rows != 1)
    {
      return false;
    }
    
    return true;
  }
  
  bool ConvertToVec4f(
    const cv::Mat& points1,
    const cv::Mat& points2,
    std::vector<cv::Vec4f>& matched_points)
  {
    matched_points.clear();
    
    if (!Valid2dPointCorrespondences(points1, points2))
    {
      return false;
    }
    
    size_t num_points = points1.cols;
    bool row_order = false;
    if (points1.rows > 1)
    {
      row_order = true;
      num_points = points1.rows;
    }
    
    // Put data into the correct format.
    matched_points.resize(num_points);
    if (row_order)
    {
      for (size_t i = 0; i < num_points; i++)
      {
        matched_points[i][0] = points1.at<cv::Vec2f>(i, 0)[0];
        matched_points[i][1] = points1.at<cv::Vec2f>(i, 0)[1];
        matched_points[i][2] = points2.at<cv::Vec2f>(i, 0)[0];
        matched_points[i][3] = points2.at<cv::Vec2f>(i, 0)[1];
      }
    }
    else
    {
      for (size_t i = 0; i < num_points; i++)
      {
        matched_points[i][0] = points1.at<cv::Vec2f>(0, i)[0];
        matched_points[i][1] = points1.at<cv::Vec2f>(0, i)[1];
        matched_points[i][2] = points2.at<cv::Vec2f>(0, i)[0];
        matched_points[i][3] = points2.at<cv::Vec2f>(0, i)[1];
      }
    }
    
    return true;
  }
}
