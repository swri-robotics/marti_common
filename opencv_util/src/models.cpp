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

#include <opencv_util/models.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace opencv_util
{
  bool AffineTransform2d::GetModel(const std::vector<T>& data, M& model)
  {
    if (data.size() != MIN_SIZE)
    {
      return false;
    }
    
    // TODO(malban): Test to make sure points aren't co-linear?
    
    cv::Point2f src[MIN_SIZE];
    cv::Point2f dst[MIN_SIZE];
    
    for (int32_t i = 0; i < MIN_SIZE; i++)
    {
      src[i].x = data[i][0];
      src[i].y = data[i][1];
      dst[i].x = data[i][2];
      dst[i].y = data[i][3];
    }
    
    model = cv::getAffineTransform(src, dst);
    
    return true;
  }
  
  double AffineTransform2d::GetError(const T& data, const M& model)
  {
    cv::Mat src(1, 1, CV_32FC2);
    src.at<cv::Vec2f>(0, 0) = cv::Vec2f(data[0], data[1]);
    
    cv::Mat dst;
    cv::transform(src, dst, model);
    cv::Vec3f& estimated = dst.at<cv::Vec3f>(0, 0);
    
    return std::sqrt(
      std::pow(data[2] - estimated[0], 2) + 
      std::pow(data[3] - estimated[1], 2));
  }

  bool RigidTransform2d::GetModel(const std::vector<T>& data, M& model)
  {
    if (data.size() != MIN_SIZE)
    {
      return false;
    }
    
    cv::Point2f src[MIN_SIZE];
    cv::Point2f dst[MIN_SIZE];
    
    for (int32_t i = 0; i < MIN_SIZE; i++)
    {
      src[i].x = data[i][0];
      src[i].y = data[i][1];
      dst[i].x = data[i][2];
      dst[i].y = data[i][3];
    }
    
    // Construct a x-axis for both sets.
    cv::Point2f src_x = (src[1] - src[0]) * (1.0 / cv::norm(src[1] - src[0]));
    cv::Point2f dst_x = (dst[1] - dst[0]) * (1.0 / cv::norm(dst[1] - dst[0]));
    
    // Construct a y-axis for both sets.
    cv::Point2f src_y(src_x.y, -src_x.x);
    src_y *= 1.0 / cv::norm(src_y);
    
    cv::Point2f dst_y(dst_x.y, -dst_x.x);
    dst_y *= 1.0 / cv::norm(dst_y);
    
    // Build rotation matrices for both sets.
    cv::Mat src_r(2, 2, CV_32F);
    src_r.at<float>(0, 0) = src_x.x;
    src_r.at<float>(1, 0) = src_x.y;
    src_r.at<float>(0, 1) = src_y.x;
    src_r.at<float>(1, 1) = src_y.y;
    
    cv::Mat dst_r(2, 2, CV_32F);
    dst_r.at<float>(0, 0) = dst_x.x;
    dst_r.at<float>(1, 0) = dst_x.y;
    dst_r.at<float>(0, 1) = dst_y.x;
    dst_r.at<float>(1, 1) = dst_y.y;
    
    // Solve for the rotation between src and dst
    //    R R_src = R_dst
    //    R = R_dst R_src^T
    cv::Mat rotation = dst_r * src_r.t();
    
    // Calculate the translation between src (rotated) and dst.
    cv::Mat src0_rotated(2, 1, CV_32F);
    src0_rotated.at<float>(0, 0) = src[0].x;
    src0_rotated.at<float>(1, 0) = src[0].y;
    src0_rotated = rotation * src0_rotated;
    float t_x = dst[0].x - src0_rotated.at<float>(0, 0);
    float t_y = dst[0].y - src0_rotated.at<float>(1, 0);
    
    model.create(2, 3, CV_32F);
    model.at<float>(0, 0) = rotation.at<float>(0, 0);
    model.at<float>(0, 1) = rotation.at<float>(0, 1);
    model.at<float>(1, 0) = rotation.at<float>(1, 0);
    model.at<float>(1, 1) = rotation.at<float>(1, 1);
    model.at<float>(0, 2) = t_x;
    model.at<float>(1, 2) = t_y;
    
    return true;
  }
  
  double RigidTransform2d::GetError(const T& data, const M& model)
  {
    cv::Mat src(1, 1, CV_32FC2);
    src.at<cv::Vec2f>(0, 0) = cv::Vec2f(data[0], data[1]);
    
    cv::Mat dst;
    cv::transform(src, dst, model);
    cv::Vec3f& estimated = dst.at<cv::Vec3f>(0, 0);
    
    return std::sqrt(
      std::pow(data[2] - estimated[0], 2) + 
      std::pow(data[3] - estimated[1], 2));
  }
  
  bool Translation2d::GetModel(const std::vector<T>& data, M& model)
  {
    if (data.size() != MIN_SIZE)
    {
      return false;
    }
  
    cv::Point2f src;
    cv::Point2f dst;
    
    src.x = data[0][0];
    src.y = data[0][1];
    dst.x = data[0][2];
    dst.y = data[0][3];
    
    // Calculate the translation between src (rotated) and dst.
    float t_x = dst.x - src.x;
    float t_y = dst.y - src.y;
    
    model.create(2, 3, CV_32F);
    model.at<float>(0, 0) = 1.0f;
    model.at<float>(0, 1) = 0.0f;
    model.at<float>(1, 0) = 0.0f;
    model.at<float>(1, 1) = 1.0f;
    model.at<float>(0, 2) = t_x;
    model.at<float>(1, 2) = t_y;
    
    return true;
  }
  
  double Translation2d::GetError(const T& data, const M& model)
  {
    cv::Mat src(1, 1, CV_32FC2);
    src.at<cv::Vec2f>(0, 0) = cv::Vec2f(data[0], data[1]);
    
    cv::Mat dst;
    cv::transform(src, dst, model);
    cv::Vec3f& estimated = dst.at<cv::Vec3f>(0, 0);
    
    return std::sqrt(
      std::pow(data[2] - estimated[0], 2) + 
      std::pow(data[3] - estimated[1], 2));
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
