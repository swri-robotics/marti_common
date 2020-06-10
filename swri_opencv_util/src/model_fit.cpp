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

#include <swri_opencv_util/model_fit.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace swri_opencv_util
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
    swri_math_util::RandomGeneratorPtr rng)
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
    swri_math_util::RandomGeneratorPtr rng)
  {
    return FindModel2d<RigidTransform2d>(
      points1, points2, inliers1, inliers2, good_points, iterations, max_error,
      confidence, max_iterations, rng);
  }

  cv::Mat FitRigidTransform2d(const cv::Mat& points1, const cv::Mat& points2)
  {
    cv::Mat transform;

    if (!Valid2dPointCorrespondences(points1, points2))
    {
      return transform;
    }

    // The Kabsch algorithm, for calculating the optimal rotation matrix that
    // minimizes the RMSD (root mean squared deviation) between two paired sets
    // of points.  http://en.wikipedia.org/wiki/Kabsch_algorithm

    // Get the centroids of the points.
    cv::Scalar centroid1 = cv::mean(points1);
    cv::Scalar centroid2 = cv::mean(points2);

    // Center the points around the origin.
    cv::Mat points1_centered = (points1 - centroid1);
    cv::Mat points2_centered = (points2 - centroid2);

    // Reshape the points into 2xN matrices.
    points1_centered = points1_centered.reshape(1, points1.rows);
    points2_centered = points2_centered.reshape(1, points1.rows);

    // Compute the covariance matrix.
    cv::Mat cov = points1_centered.t() * points2_centered;

    // Compute the optimal rotation matrix.
    cv::Mat W, U, Vt;
    cv::SVD::compute(cov, W, U, Vt);
    double d = cv::determinant(Vt.t() * U.t()) > 0 ? 1.0 : -1.0;
    cv::Mat I = cv::Mat::eye(2, 2, CV_32F);
    I.at<float>(1, 1) = d;
    cv::Mat rotation = Vt.t() * I * U.t();

    // Compute the optimal translation.
    cv::Mat c1_r(2, 1, CV_32F);
    c1_r.at<float>(0, 0) = centroid1[0];
    c1_r.at<float>(1, 0) = centroid1[1];
    c1_r = rotation * c1_r;
    float t_x = centroid2[0] - c1_r.at<float>(0, 0);
    float t_y = centroid2[1] - c1_r.at<float>(1, 0);

    transform.create(2, 3, CV_32F);
    transform.at<float>(0, 0) = rotation.at<float>(0, 0);
    transform.at<float>(0, 1) = rotation.at<float>(0, 1);
    transform.at<float>(1, 0) = rotation.at<float>(1, 0);
    transform.at<float>(1, 1) = rotation.at<float>(1, 1);
    transform.at<float>(0, 2) = t_x;
    transform.at<float>(1, 2) = t_y;

    return transform;
  }

  cv::Mat FitRotation3d(const cv::Mat& points1, const cv::Mat& points2)
  {
    cv::Mat matrix;

    if (!Valid3dPointCorrespondences(points1, points2))
    {
      return matrix;
    }

    // The Kabsch algorithm, for calculating the optimal rotation matrix that
    // minimizes the RMSD (root mean squared deviation) between two paired sets
    // of points.  http://en.wikipedia.org/wiki/Kabsch_algorithm

    // Compute the covariance matrix.
    cv::Mat src = points1.reshape(1, points1.rows);
    cv::Mat dst = points2.reshape(1, points1.rows);

    cv::Mat cov = src.t() * dst;

    // Compute the optimal rotation matrix.
    cv::Mat W, U, Vt;
    cv::SVD::compute(cov, W, U, Vt);
    cv::Mat V = Vt.t();
    double d = cv::determinant(V * U.t()) > 0 ? 1.0 : -1.0;
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    I.at<float>(2, 2) = d;
    matrix = V * I * U.t();

    return matrix;
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
    swri_math_util::RandomGeneratorPtr rng)
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
    swri_math_util::RandomGeneratorPtr rng)
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
    swri_math_util::RandomGeneratorPtr rng)
  {
    swri_math_util::Ransac<PerpendicularPlaneWithPointFit> ransac(rng);

    cv::Mat reshaped = points.reshape(3);
    PerpendicularPlaneWithPointFit fit_model(reshaped, point_on_plane, perp_axis, max_angle_from_perp);
    PlaneModel model = ransac.FitModel(
      fit_model, max_error, confidence, min_iterations, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    inliers = cv::Mat(good_points.size(), reshaped.cols, reshaped.type());
    for (size_t i = 0; i < good_points.size(); ++i)
    {
      inliers.at<cv::Vec3f>(i, 0) = reshaped.at<cv::Vec3f>(good_points[i], 0);
    }
    inliers.reshape(points.channels());
    return model;
  }


  PlaneModel FindPlane(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t min_iterations,
    int32_t max_iterations,
    swri_math_util::RandomGeneratorPtr rng)
  {
    swri_math_util::Ransac<PlaneFit> ransac(rng);

    cv::Mat reshaped = points.reshape(3);
    PlaneFit fit_model(reshaped);
    PlaneModel model = ransac.FitModel(
      fit_model, max_error, confidence, min_iterations, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    inliers = cv::Mat(good_points.size(), reshaped.cols, reshaped.type());
    for (size_t i = 0; i < good_points.size(); ++i)
    {
      inliers.at<cv::Vec3f>(i, 0) = reshaped.at<cv::Vec3f>(good_points[i], 0);
    }
    inliers.reshape(points.channels());
    return model;
  }

  PlaneModel FitPlane(const cv::Mat& points)
  {
    PlaneModel model;
    if (points.rows < 3)
    {
      return model;
    }
    cv::Mat centroid;
    cv::reduce(points.reshape(3), centroid, 0, cv::REDUCE_AVG);

    cv::Scalar c(centroid.at<float>(0, 0), centroid.at<float>(0, 1), centroid.at<float>(0, 2));

    cv::Mat A;
    cv::subtract(points, c, A);

    cv::SVD svd;
    cv::Mat w, u, vt;
    cv::Mat At;
    cv::transpose(A.reshape(1), At);
    svd.compute(At, w, u, vt);

    cv::Point min_w_loc;
    cv::minMaxLoc(w, NULL, NULL, &min_w_loc);

    model.x = centroid.at<float>(0, 0);
    model.y = centroid.at<float>(0, 1);
    model.z = centroid.at<float>(0, 2);
    model.i = u.at<float>(0, min_w_loc.y);
    model.j = u.at<float>(1, min_w_loc.y);
    model.k = u.at<float>(2, min_w_loc.y);

    return model;
  }

  LineModel3d FindLine3d(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t min_iterations,
    int32_t max_iterations,
    swri_math_util::RandomGeneratorPtr rng)
  {
    swri_math_util::Ransac<LineFit3d> ransac(rng);

    cv::Mat reshaped = points.reshape(3);
    LineFit3d fit_model(reshaped);
    LineModel3d model = ransac.FitModel(
      fit_model, max_error, confidence, min_iterations, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    inliers = cv::Mat(good_points.size(), reshaped.cols, reshaped.type());
    for (size_t i = 0; i < good_points.size(); ++i)
    {
      inliers.at<cv::Vec3f>(i, 0) = reshaped.at<cv::Vec3f>(good_points[i], 0);
    }
    inliers.reshape(points.channels());
    return model;
  }

  LineModel3d FindOrthoLine3d(
    const cv::Mat& points,
    const LineModel3d& ortho,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t min_iterations,
    int32_t max_iterations,
    swri_math_util::RandomGeneratorPtr rng)
  {
    swri_math_util::Ransac<LineFit3d> ransac(rng);

    cv::Mat reshaped = points.reshape(3);
    OrthoLineFit3d fit_model(reshaped, ortho);
    LineModel3d model = ransac.FitModel(
      fit_model, max_error, confidence, min_iterations, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    inliers = cv::Mat(good_points.size(), reshaped.cols, reshaped.type());
    for (size_t i = 0; i < good_points.size(); ++i)
    {
      inliers.at<cv::Vec3f>(i, 0) = reshaped.at<cv::Vec3f>(good_points[i], 0);
    }
    inliers.reshape(points.channels());
    return model;
  }

  LineModel3d FitLine3d(const cv::Mat& points)
  {
    LineModel3d model;
    if (points.rows < 2)
    {
      return model;
    }
    cv::Mat centroid;
    cv::reduce(points.reshape(3), centroid, 0, cv::REDUCE_AVG);

    cv::Scalar c(centroid.at<float>(0, 0), centroid.at<float>(0, 1), centroid.at<float>(0, 2));

    cv::Mat A;
    cv::subtract(points, c, A);

    cv::SVD svd;
    cv::Mat w, u, vt;
    cv::Mat At;
    cv::transpose(A.reshape(1), At);
    svd.compute(At, w, u, vt);

    cv::Point max_w_loc;
    cv::minMaxLoc(w, NULL, NULL, NULL, &max_w_loc);

    model.x = centroid.at<float>(0, 0);
    model.y = centroid.at<float>(0, 1);
    model.z = centroid.at<float>(0, 2);
    model.i = u.at<float>(0, max_w_loc.y);
    model.j = u.at<float>(1, max_w_loc.y);
    model.k = u.at<float>(2, max_w_loc.y);

    return model;
  }

  CrossModel3d FindCross3d(
    const cv::Mat& points,
    cv::Mat& inliers,
    std::vector<uint32_t> &good_points,
    int32_t& iterations,
    double max_error,
    double confidence,
    int32_t min_iterations,
    int32_t max_iterations,
    swri_math_util::RandomGeneratorPtr rng)
  {
    swri_math_util::Ransac<CrossFit3d> ransac(rng);

    cv::Mat reshaped = points.reshape(3);
    CrossFit3d fit_model(reshaped);
    CrossModel3d model = ransac.FitModel(
      fit_model, max_error, confidence, min_iterations, max_iterations, good_points, iterations);

    if (good_points.empty())
    {
      return model;
    }

    inliers = cv::Mat(good_points.size(), reshaped.cols, reshaped.type());
    for (size_t i = 0; i < good_points.size(); ++i)
    {
      inliers.at<cv::Vec3f>(i, 0) = reshaped.at<cv::Vec3f>(good_points[i], 0);
    }
    inliers.reshape(points.channels());
    return model;
  }
}
