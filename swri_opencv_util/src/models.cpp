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

#include <swri_opencv_util/models.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace swri_opencv_util
{

  int32_t Correspondence2d::GetInlierCount(const M& model, double max_error)
  {
    CalculateNorms(model, norms__);

    cv::compare(norms__, cv::Scalar(max_error * max_error), thresholded__, cv::CMP_LT);

    return cv::countNonZero(thresholded__);
  }
  
  void Correspondence2d::GetInliers(const M& model, double max_error, std::vector<uint32_t>& indices)
  {
    CalculateNorms(model, norms__);

    indices.clear();
    indices.reserve(norms__.rows);
    double threshold = max_error * max_error;
    for (int i = 0; i < norms__.rows; i++)
    {
      if (norms__.at<float>(i) < threshold)
      {
        indices.push_back(i);
      }
    }
  }

  void Correspondence2d::CalculateNorms(const M& model, cv::Mat& norms)
  {
    cv::Mat src = data_(cv::Rect(0, 0, 2, data_.rows)).reshape(2);
    cv::transform(src, predicted__, model);
    cv::Mat measured = data_(cv::Rect(2, 0, 2, data_.rows));
    cv::subtract(predicted__.reshape(1), measured, delta__);
    cv::multiply(delta__, delta__, delta_squared__);
    cv::add(
      delta_squared__(cv::Rect(0, 0, 1, delta__.rows)), 
      delta_squared__(cv::Rect(1, 0, 1, delta__.rows)),
      norms);
  }

  bool Homography::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }
    
    cv::Mat src(MIN_SIZE, 1, CV_32FC2);
    cv::Mat dst(MIN_SIZE, 1, CV_32FC2);
    
    for (int32_t i = 0; i < MIN_SIZE; i++)
    {
      const float* sample = data_.ptr<float>(indices[i]);
      src.at<cv::Vec2f>(i, 0) = cv::Vec2f(sample[0], sample[1]);
      dst.at<cv::Vec2f>(i, 0) = cv::Vec2f(sample[2], sample[3]);
    }
    
    model = cv::getPerspectiveTransform(src, dst);
    
    // Test input points for if they all match the generated model.
    cv::Mat predicted;
    cv::perspectiveTransform(src, predicted, model);
    cv::Mat delta, delta_squared, norms;
    cv::subtract(predicted.reshape(1), dst.reshape(1), delta);
    cv::multiply(delta, delta, delta_squared);
    cv::add(
      delta_squared(cv::Rect(0, 0, 1, delta.rows)), 
      delta_squared(cv::Rect(1, 0, 1, delta.rows)),
      norms);
      
    double min, max;
    cv::minMaxLoc(norms, &min, &max);
    
    return max < max_error * max_error;
  }
  
  void Homography::CalculateNorms(const M& model, cv::Mat& norms)
  {
    cv::Mat src = data_(cv::Rect(0, 0, 2, data_.rows)).reshape(2);
    cv::perspectiveTransform(src, predicted__, model);
    cv::Mat measured = data_(cv::Rect(2, 0, 2, data_.rows));
    cv::subtract(predicted__.reshape(1), measured, delta__);
    cv::multiply(delta__, delta__, delta_squared__);
    cv::add(
      delta_squared__(cv::Rect(0, 0, 1, delta__.rows)), 
      delta_squared__(cv::Rect(1, 0, 1, delta__.rows)),
      norms);
  }
  
  bool AffineTransform2d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }
    
    cv::Mat src(MIN_SIZE, 1, CV_32FC2);
    cv::Mat dst(MIN_SIZE, 1, CV_32FC2);
    
    for (int32_t i = 0; i < MIN_SIZE; i++)
    {
      const float* sample = data_.ptr<float>(indices[i]);
      src.at<cv::Vec2f>(i, 0) = cv::Vec2f(sample[0], sample[1]);
      dst.at<cv::Vec2f>(i, 0) = cv::Vec2f(sample[2], sample[3]);
    }
    
    model = cv::getAffineTransform(src, dst);
    
    // Test input points for if they all match the generated model.
    cv::Mat predicted;
    cv::transform(src, predicted, model);
    cv::Mat delta, delta_squared, norms;
    cv::subtract(predicted.reshape(1), dst.reshape(1), delta);
    cv::multiply(delta, delta, delta_squared);
    cv::add(
      delta_squared(cv::Rect(0, 0, 1, delta.rows)), 
      delta_squared(cv::Rect(1, 0, 1, delta.rows)),
      norms);
      
    double min, max;
    cv::minMaxLoc(norms, &min, &max);
    
    return max < max_error * max_error;
  }

  bool RigidTransform2d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }
    
    cv::Point2f src[MIN_SIZE];
    cv::Point2f dst[MIN_SIZE];
    
    for (int32_t i = 0; i < MIN_SIZE; i++)
    {
      const float* sample = data_.ptr<float>(indices[i]);
      src[i].x = sample[0];
      src[i].y = sample[1];
      dst[i].x = sample[2];
      dst[i].y = sample[3];
    }
    
    double len_src = cv::norm(src[1] - src[0]);
    double len_dst = cv::norm(dst[1] - dst[0]);
    if (std::fabs(len_src - len_dst) >= max_error)
    {
      return false;
    }
    
    // Construct a x-axis for both sets.
    cv::Point2f src_x = (src[1] - src[0]) * (1.0 / len_src);
    cv::Point2f dst_x = (dst[1] - dst[0]) * (1.0 / len_dst);
    
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
  
  bool Translation2d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }
  
    cv::Point2f src;
    cv::Point2f dst;
    
    const float* sample = data_.ptr<float>(indices[0]);
    src.x = sample[0];
    src.y = sample[1];
    dst.x = sample[2];
    dst.y = sample[3];
    
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
  
  bool ZipCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& correspondeces)
  {    
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
    if (row_order)
    {
      cv::hconcat(points1.reshape(1), points2.reshape(1), correspondeces);
    }
    else
    {
      cv::hconcat(
        points1.reshape(0, num_points).reshape(1), 
        points2.reshape(0, num_points).reshape(1),
        correspondeces);
    }
    
    return true;
  }
}
