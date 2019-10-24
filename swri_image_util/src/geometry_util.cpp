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

#include <swri_image_util/geometry_util.h>

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <swri_geometry_util/intersection.h>
#include <swri_math_util/constants.h>

namespace swri_image_util
{
  bool Intersects(const BoundingBox& box1, const BoundingBox& box2)
  {
    return (box1 & box2).area() > 0;
  }

  double GetOverlappingArea(const cv::Rect& rect, const cv::Mat& rigid_transform)
  {
    // List of points corresponding to the input rectangle.
    std::vector<cv::Vec2d> points;

    // List of points correspondng to the transformed rectangle.
    std::vector<cv::Vec2d> points_t;

    // Create a point for each corner of the input rectangle.
    points.push_back(cv::Vec2d(rect.x, rect.y));
    points.push_back(cv::Vec2d(rect.x + rect.width, rect.y));
    points.push_back(cv::Vec2d(rect.x + rect.width, rect.y + rect.height));
    points.push_back(cv::Vec2d(rect.x, rect.y + rect.height));

    // Transform the input points to the transformed points using the rigid
    // transform.
    cv::transform(cv::InputArray(points), cv::OutputArray(points_t), rigid_transform);

    // Use swri_geometry_util to get the overlapping area
    return swri_geometry_util::PolygonIntersectionArea(points, points_t);
  }

  cv::Mat ProjectEllipsoid(const cv::Mat& ellipsoid)
  {
    cv::Mat ellipse;

    if (ellipsoid.rows == 3 && ellipsoid.cols == 3 && ellipsoid.type() == CV_32FC1)
    {
      if (ellipsoid.at<float>(2, 2) >= std::numeric_limits<double>::max() * 0.5)
      {
        ellipse.create(2, 2, CV_32FC1);
        ellipse.at<float>(0, 0) = ellipsoid.at<float>(0, 0);
        ellipse.at<float>(0, 1) = ellipsoid.at<float>(0, 1);
        ellipse.at<float>(1, 0) = ellipsoid.at<float>(1, 0);
        ellipse.at<float>(1, 1) = ellipsoid.at<float>(1, 1);

        return ellipse;
      }

      Eigen::Matrix3d A;
      for (size_t r = 0; r < 3; r++)
      {
        for (size_t c = 0; c < 3; c++)
        {
          A(r, c) = ellipsoid.at<float>(r, c);
        }
      }

      // Specify the main vector directions (x and y)
      Eigen::Vector3d ax1;
      Eigen::Vector3d ax2;
      ax1 << 1, 0, 0;
      ax2 << 0, 1, 0;

      // Initialize the normal vector (here the normal vector, in the state
      // space is in the theta direction).
      Eigen::Vector3d n_ax;
      n_ax << 0, 0, 1;

      ///////////////////////
      // Calculate A prime //
      ///////////////////////

      Eigen::Matrix3d A_sym_temp = A.transpose() + A;

      // N_ax_temp = (n_ax*n_ax')
      Eigen::Matrix3d N_ax_temp = n_ax * n_ax.transpose();

      // A_prime_1 = A_sym_temp*N_ax_temp
      //           = (A+A')*(n_ax*n_ax')
      Eigen::Matrix3d A_prime_1 = A_sym_temp * N_ax_temp;

      // A_prime_2 = A_prime_1*A_sym_temp
      //           = (A+A')*(n_ax*n_ax')*(A+A')
      Eigen::Matrix3d A_prime_2 = A_prime_1 * A_sym_temp;

      // scale_1 = n_ax'*A
      Eigen::RowVector3d scale_1 = n_ax.transpose() * A;

      // scale_2 = (scale_1*n_ax)*-4
      //         = (n_ax'*A*n_ax)*-4
      double scale = (scale_1 * n_ax)(0,0) * -4.0;

      // A_temp = scale*A_temp
      //        = scale_2*A = -4*(n_ax'*A*n_ax)*A
      Eigen::Matrix3d A_temp = A * scale;

      // Aprime = A_prime_2 + A_temp
      //        = (A+A')*(n_ax*n_ax')*(A+A') - 4*(n_ax'*A*n_ax)*A
      Eigen::Matrix3d Aprime = A_prime_2 + A_temp;

      ///////////////////////
      // Calculate C prime //
      ///////////////////////

      // C_temp = n_ax'*A
      Eigen::RowVector3d C_temp = n_ax.transpose() * A;

      // Cprime = -4.0*C_temp*n_ax
      //        = -4.0*n_ax'*A*n_ax
      double cp = (-4.0 * C_temp) * n_ax;

      // Bprime = Aprime/Cprime;
      Eigen::Matrix3d Bprime = Aprime / cp;

      // Jp = axes_def;
      //    = [ax1(:),ax2(:)] = [1,0;0,1;0,0];
      Eigen::Matrix<double, 3, 2> Jp;
      Jp << 1, 0,
            0, 1,
            0, 0;

      ///////////////////////
      // Calculate D prime //
      ///////////////////////

      // Dprime_temp = Jp'*Bprime
      Eigen::Matrix<double, 2, 3> Dprime_temp = Jp.transpose() * Bprime;

      // Dprime = Dprime_temp * Jp
      //        = Jp'*Bprime*Jp
      Eigen::Matrix2d Dprime = Dprime_temp * Jp;

      for (size_t r = 0; r < 2; r++)
      {
        for (size_t c = 0; c < 2; c++)
        {
          if (Dprime(r, c) != Dprime(r, c))
          {
            return ellipse;
          }
        }
      }

      ellipse.create(2, 2, CV_32FC1);
      ellipse.at<float>(0, 0) = Dprime(0, 0);
      ellipse.at<float>(0, 1) = Dprime(0, 1);
      ellipse.at<float>(1, 0) = Dprime(1, 0);
      ellipse.at<float>(1, 1) = Dprime(1, 1);
    }

    return ellipse;
  }

  std::vector<tf2::Vector3> GetEllipsePoints(
      const cv::Mat& ellipse,
      const tf2::Vector3& center,
      double scale,
      int32_t num_points)
  {
    std::vector<tf2::Vector3> perimeter;

    if (ellipse.rows == 2 && ellipse.cols == 2 && ellipse.type() == CV_32FC1 &&
        num_points > 2)
    {
      Eigen::Matrix2d Dprime;
      Dprime(0, 0) = ellipse.at<float>(0, 0);
      Dprime(0, 1) = ellipse.at<float>(0, 1);
      Dprime(1, 0) = ellipse.at<float>(1, 0);
      Dprime(1, 1) = ellipse.at<float>(1, 1);

      Eigen::JacobiSVD<Eigen::Matrix2d> svd(Dprime, Eigen::ComputeFullV);

      Eigen::Vector2d Sigma = svd.singularValues();
      Eigen::Matrix2d Vt = svd.matrixV().transpose();

      double xprime_scale = std::sqrt(Sigma(0));
      double yprime_scale = std::sqrt(Sigma(1));

      if (xprime_scale <= 0 || yprime_scale <= 0)
      {
        return perimeter;
      }

      Eigen::MatrixX2d Xp1(num_points, 2);
      for (int32_t i = 0; i < num_points; i++)
      {
        double phi =
            (static_cast<double>(i) / static_cast<double>(num_points))
            * swri_math_util::_2pi;

        Xp1(i, 0) = xprime_scale * std::cos(phi) * scale;
        Xp1(i, 1) = yprime_scale * std::sin(phi) * scale;
      }

      // Xell = Xp1*(V')'
      Eigen::MatrixX2d Xell = Xp1 * Vt;

      perimeter.resize(num_points);
      for (int32_t i = 0; i < num_points; i++)
      {
        perimeter[i].setX(Xell(i, 0) + center.x());
        perimeter[i].setY(Xell(i, 1) + center.y());
      }
    }

    return perimeter;
  }
}
