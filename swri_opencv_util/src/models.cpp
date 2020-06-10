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

  bool Valid3dPointCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2)
  {
    if (points1.type() != points2.type())
    {
      return false;
    }

    if (points1.type() != CV_32FC3)
    {
      return false;
    }

    if (points1.cols != points2.cols  || points1.rows  != points2.rows)
    {
      return false;
    }

    if (points1.cols != 1)
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


  bool PerpendicularPlaneWithPointFit::GetModel(const std::vector<int32_t>& indices, M& model,
                                                double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }

    // Check if points are collinear.

    cv::Mat points = data_.reshape(3);

    cv::Vec3f p1 = points.at<cv::Vec3f>(indices[0], 0);
    cv::Vec3f p2 = points.at<cv::Vec3f>(indices[1], 0);
    cv::Vec3f p3 = point_;

    cv::Point3f v12 = p2 - p1;
    cv::Point3f v13 = p3 - p1;
    float d12 = cv::norm(v12);
    float d13 = cv::norm(v13);
    float d = std::fabs(d12 * d13);
    if (d == 0)
    {
      return false;
    }

    float angle = std::acos(v12.dot(v13) / d);
    if (angle < min_angle_ || angle + min_angle_ > M_PI)
    {
      return false;
    }

    // Calculate model.

    cv::Vec3f normal = v12.cross(v13);
    normal = normal / cv::norm(normal);

    if (std::acos(normal.dot(perp_axis_)) > max_axis_angle_)
    {
      return false;
    }

    model.x = p1[0];
    model.y = p1[1];
    model.z = p1[2];
    model.i = normal[0];
    model.j = normal[1];
    model.k = normal[2];

    return true;
  }

  bool PlaneFit::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }

    // Check if points are collinear.

    cv::Mat points = data_.reshape(3);

    cv::Vec3f p1 = points.at<cv::Vec3f>(indices[0], 0);
    cv::Vec3f p2 = points.at<cv::Vec3f>(indices[1], 0);
    cv::Vec3f p3 = points.at<cv::Vec3f>(indices[2], 0);

    cv::Point3f v12 = p2 - p1;
    cv::Point3f v13 = p3 - p1;
    float d12 = cv::norm(v12);
    float d13 = cv::norm(v13);
    float d = std::fabs(d12 * d13);
    if (d == 0)
    {
      return false;
    }

    float angle = std::acos(v12.dot(v13) / d);
    if (angle < min_angle_ || angle + min_angle_ > M_PI)
    {
      return false;
    }

    // Calculate model.

    cv::Vec3f normal = v12.cross(v13);
    normal = normal / cv::norm(normal);

    model.x = p1[0];
    model.y = p1[1];
    model.z = p1[2];
    model.i = normal[0];
    model.j = normal[1];
    model.k = normal[2];

    return true;
  }

  void PlaneFit::CalculateNorms(const M& model, cv::Mat& norms)
  {
    // Subtract the origin point of the plane from each data point
    cv::Mat single_column = data_.reshape(3);
    cv::subtract(single_column, cv::Scalar(model.x, model.y, model.z), delta__);

    // Calculate the dot product of each adjusted data point with the normal
    // of the plane.
    cv::Mat split_columns = delta__.reshape(1);
    cv::Mat x = split_columns(cv::Rect(0, 0, 1, split_columns.rows));
    cv::Mat y = split_columns(cv::Rect(1, 0, 1, split_columns.rows));
    cv::Mat z = split_columns(cv::Rect(2, 0, 1, split_columns.rows));
    x = x * model.i;
    y = y * model.j;
    z = z * model.k;
    cv::add(x, y, norms);
    cv::add(norms, z, norms);
    norms = cv::abs(norms);
  }

  bool LineFit3d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }

    cv::Mat points = data_.reshape(3);

    cv::Point3f p1(points.at<cv::Vec3f>(indices[0], 0));
    cv::Point3f p2(points.at<cv::Vec3f>(indices[1], 0));

    cv::Point3f v12 = p2 - p1;
    v12 = cv::Vec3f(v12) / cv::norm(v12);

    model.x = p1.x;
    model.y = p1.y;
    model.z = p1.z;
    model.i = v12.x;
    model.j = v12.y;
    model.k = v12.z;

    return true;
  }

  void LineFit3d::CalculateNorms(const M& model, cv::Mat& norms)
  {
    // d = ||(x0 - p) - ((x0 - p) . n)n||

    cv::Scalar x0(model.x, model.y, model.z);
    cv::Scalar n(model.i, model.j, model.k);
    cv::Mat p = data_.reshape(3);

    if (temp1__.size != p.size)
    {
      temp1__ = cv::Mat(p.rows, p.cols, p.type());
    }
    temp1__.setTo(n);
    cv::Mat n_columns = temp1__.reshape(1);
    cv::Mat n_x = n_columns(cv::Rect(0, 0, 1, n_columns.rows));
    cv::Mat n_y = n_columns(cv::Rect(1, 0, 1, n_columns.rows));
    cv::Mat n_z = n_columns(cv::Rect(2, 0, 1, n_columns.rows));

    // (x0 - p)
    cv::subtract(p, x0, x0_p__);

    // (x0 - p) . n
    cv::multiply(x0_p__, temp1__, temp2__);
    cv::reduce(temp2__.reshape(1), x0_p_dot_n__, 1, cv::REDUCE_SUM);

    // ((x0 - p) . n)n
    cv::multiply(n_x, x0_p_dot_n__, n_x);
    cv::multiply(n_y, x0_p_dot_n__, n_y);
    cv::multiply(n_z, x0_p_dot_n__, n_z);

    // (x0 - p) - ((x0 - p) . n)n
    cv::subtract(x0_p__, temp1__, temp1__);

    // d = ||(x0 - p) - ((x0 - p) . n)n||
    cv::multiply(n_x, n_x, n_x);
    cv::multiply(n_y, n_y, n_y);
    cv::multiply(n_z, n_z, n_z);
    cv::reduce(temp1__.reshape(1), norms, 1, cv::REDUCE_SUM);
    cv::sqrt(norms, norms);
  }

  bool OrthoLineFit3d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }

    cv::Mat points = data_.reshape(3);

    cv::Point3f p1(points.at<cv::Vec3f>(indices[0], 0));
    cv::Point3f p2(points.at<cv::Vec3f>(indices[1], 0));

    cv::Point3f v12 = p2 - p1;
    v12 = cv::Vec3f(v12) / cv::norm(v12);

    // Check if orthogonal
    cv::Point3f ortho(ortho_.i, ortho_.j, ortho_.k);
    ortho = cv::Vec3f(ortho) / cv::norm(ortho);

    float angle = std::acos(v12.dot(ortho));
    float error = std::fabs(M_PI * 0.5 - angle);
    if (error > angle_tolerance_)
    {
      return false;
    }

    model.x = p1.x;
    model.y = p1.y;
    model.z = p1.z;
    model.i = v12.x;
    model.j = v12.y;
    model.k = v12.z;

    return true;
  }

  bool CrossFit3d::GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const
  {
    if (indices.size() != MIN_SIZE)
    {
      return false;
    }

    // Check if points are collinear.

    cv::Mat points = data_.reshape(3);

    cv::Point3f p1(points.at<cv::Vec3f>(indices[0], 0));
    cv::Point3f p2(points.at<cv::Vec3f>(indices[1], 0));
    cv::Point3f p3(points.at<cv::Vec3f>(indices[2], 0));

    cv::Point3f v12 = p2 - p1;
    cv::Point3f v13 = p3 - p1;
    float d12 = cv::norm(v12);
    float d13 = cv::norm(v13);
    float d = std::fabs(d12 * d13);
    if (d == 0)
    {
      return false;
    }

    float angle = std::acos(v12.dot(v13) / d);
    if (angle < min_angle_ || angle + min_angle_ > M_PI)
    {
      return false;
    }

    // Calculate model.

    // Orthogonally project 3rd point onto first line to
    // get line through 3rd point orthogonal to the first
    // line.
    cv::Point3f p4 = p1 + (v13.dot(v12) / v12.dot(v12)) * v12;
    cv::Point3f v34 = p4 - p3;

    // Get unit vector of the lines.
    v12 = cv::Vec3f(v12) / cv::norm(v12);
    v34 = cv::Vec3f(v34) / cv::norm(v34);

    model.x = p4.x;
    model.y = p4.y;
    model.z = p4.z;
    model.i1 = v12.x;
    model.j1 = v12.y;
    model.k1 = v12.z;
    model.i2 = v34.x;
    model.j2 = v34.y;
    model.k2 = v34.z;

    return true;
  }

  void CrossFit3d::CalculateNorms(const M& model, cv::Mat& norms)
  {
    //////////////////////////////////////////
    // Line 1
    //////////////////////////////////////////

    // d = ||(x0 - p) - ((x0 - p) . n)n||

    cv::Scalar x0(model.x, model.y, model.z);
    cv::Scalar n(model.i1, model.j1, model.k1);
    cv::Mat p = data_.reshape(3);

    if (temp1__.size != p.size)
    {
      temp1__ = cv::Mat(p.rows, p.cols, p.type());
    }
    temp1__.setTo(n);
    cv::Mat n_columns = temp1__.reshape(1);
    cv::Mat n_x = n_columns(cv::Rect(0, 0, 1, n_columns.rows));
    cv::Mat n_y = n_columns(cv::Rect(1, 0, 1, n_columns.rows));
    cv::Mat n_z = n_columns(cv::Rect(2, 0, 1, n_columns.rows));

    // (x0 - p)
    cv::subtract(p, x0, x0_p__);

    // (x0 - p) . n
    cv::multiply(x0_p__, temp1__, temp2__);
    cv::reduce(temp2__.reshape(1), x0_p_dot_n__, 1, cv::REDUCE_SUM);

    // ((x0 - p) . n)n
    cv::multiply(n_x, x0_p_dot_n__, n_x);
    cv::multiply(n_y, x0_p_dot_n__, n_y);
    cv::multiply(n_z, x0_p_dot_n__, n_z);

    // (x0 - p) - ((x0 - p) . n)n
    cv::subtract(x0_p__, temp1__, temp1__);

    // d = ||(x0 - p) - ((x0 - p) . n)n||
    cv::multiply(n_x, n_x, n_x);
    cv::multiply(n_y, n_y, n_y);
    cv::multiply(n_z, n_z, n_z);
    cv::reduce(temp1__.reshape(1), temp3__, 1, cv::REDUCE_SUM);
    cv::sqrt(temp3__, temp3__);


    //////////////////////////////////////////
    // Line 2
    //////////////////////////////////////////

    // d = ||(x0 - p) - ((x0 - p) . n)n||

    n = cv::Scalar(model.i2, model.j2, model.k2);
    temp1__.setTo(n);

    // (x0 - p)
    cv::subtract(p, x0, x0_p__);

    // (x0 - p) . n
    cv::multiply(x0_p__, temp1__, temp2__);
    cv::reduce(temp2__.reshape(1), x0_p_dot_n__, 1, cv::REDUCE_SUM);

    // ((x0 - p) . n)n
    cv::multiply(n_x, x0_p_dot_n__, n_x);
    cv::multiply(n_y, x0_p_dot_n__, n_y);
    cv::multiply(n_z, x0_p_dot_n__, n_z);

    // (x0 - p) - ((x0 - p) . n)n
    cv::subtract(x0_p__, temp1__, temp1__);

    // d = ||(x0 - p) - ((x0 - p) . n)n||
    cv::multiply(n_x, n_x, n_x);
    cv::multiply(n_y, n_y, n_y);
    cv::multiply(n_z, n_z, n_z);
    cv::reduce(temp1__.reshape(1), norms, 1, cv::REDUCE_SUM);
    cv::sqrt(norms, norms);

    // Use the minimum distance to either line.
    norms = cv::min(norms, temp3__);
  }
}
