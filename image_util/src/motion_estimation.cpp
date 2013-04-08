// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
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
/*
 * motion_estimation.cpp
 *
 *  Created on: Jan 19, 2012
 *      Author: kkozak
 */

#include <image_util/motion_estimation.h>

#include <algorithm>

namespace image_util
{
  cv::Mat LeastSqauresRigid2DTransform(
    const cv::Mat& inliers1,
    const cv::Mat& inliers2)
  {
    cv::Mat T;

    // Verify that the input points are well formed.
    if (!ValidPointsForTransform(inliers1, inliers2))
    {
      ROS_ERROR("Invalid points for calculating transform.");
      return T;
    }

    // Determine if the points are row ordered.
    bool row_order = inliers1.rows > 1;
    int32_t num_points = row_order ? inliers1.rows : inliers1.cols;

    // Build the linear equation.
    LaGenMatDouble A(num_points, 3);
    LaGenMatDouble B(num_points, 2);
    for (int32_t i = 0; i < num_points; i++)
    {
      int32_t row = row_order ? i : 0;
      int32_t col = row_order ? 0 : i;

      A(i, 0) = inliers1.at<cv::Vec2f>(row, col)[0];
      A(i, 1) = inliers1.at<cv::Vec2f>(row, col)[1];
      A(i, 2) = 1.0;

      B(i, 0) = inliers2.at<cv::Vec2f>(row, col)[0];
      B(i, 1) = inliers2.at<cv::Vec2f>(row, col)[1];
    }

    // Solve for the Transformation matrix, X
    LaGenMatDouble X(3, 2);
    LaLinearSolve(A, X, B);
    double cn;
    double rnorm;
    RegularizeTransform(X, cn, rnorm);
    T.create(2, 3, CV_32FC1);

    for (uint32_t i = 0; i < 2; i++)
    {
      for (uint32_t j = 0; j < 3; j++)
      {
        T.at<float>(i, j) = X(j, i);
      }
    }

    return T;
  }

  bool ValidPointsForTransform(
    const cv::Mat& points1,
    const cv::Mat& points2)
  {
    if (points1.type() != points2.type())
    {
      ROS_ERROR("Mismatched array types.");
      return false;
    }

    if (points1.type() != CV_32FC2 && points1.type() != CV_32FC3)
    {
      ROS_ERROR("Array type must be either CV_32FC2 or CV_32FC3");
      return false;
    }

    if (points1.cols != points2.cols  || points1.rows  != points2.rows)
    {
      ROS_ERROR("Mismatched array sizes.");
      return false;
    }

    if (points1.cols != 1 && points1.rows != 1)
    {
      ROS_ERROR("Invalid array size: The arrays need to be 1xN or Nx1.");
      return false;
    }

    if (points1.cols < 6 && points1.rows < 6)
    {
      ROS_ERROR("At least 6 points required to calculate rigid transform.");
      return false;
    }

    return true;
  }

  double CalculateReprojectionError(
    const cv::Mat& points1,
    const cv::Mat& points2,
    const cv::Mat& transform)
  {
    // Transform the first set of points.
    cv::Mat transformed;
    cv::transform(points1, transformed, transform);

    // Measure the point-wise absolute difference between the set of transformed
    // points and the second set of points.
    cv::Mat difference;
    cv::absdiff(transformed, points2, difference);

    // Calculate the element-wise mean difference.
    cv::Scalar mean_error = cv::mean(difference);

    // Calculate the mean reprojection error as the L2-norm of the element-wise
    // mean difference.
    double error = cv::norm(mean_error);

    return error;
  }

  tf::Transform ToTransform(const cv::Mat& matrix)
  {
    tf::Transform transform = tf::Transform::getIdentity();

    if (matrix.rows == 2 && matrix.cols == 3)
    {
      transform.setOrigin(tf::Vector3(
          matrix.at<float>(0, 2),
          matrix.at<float>(1, 2),
          0.0));

      transform.getBasis().setValue(
          matrix.at<float>(0, 0),
          matrix.at<float>(0, 1),
          0.0,
          matrix.at<float>(1, 0),
          matrix.at<float>(1, 1),
          0.0,
          0.0, 0.0, 1.0);
    }

    return transform;
  }

  void PrintMatrix(const LaGenMatDouble &A)
  {
    int i, j;
    for (i = 0; i < A.rows(); i++)
    {
      fprintf(stderr, "\n");
      for (j = 0; j < A.cols(); j++)
        fprintf(stderr, "%15.10f ", A(i, j));
    }
    fprintf(stderr, "\n");
  }

  void ExtractMotionParameters(
      const cv::Mat& Fin,
      const cv::Mat& Intrinsics,
      cv::Mat& R1,
      cv::Mat& R2,
      cv::Mat& T)
  {
    if (Fin.cols != 3 || Fin.rows != 3)
    {
      ROS_ERROR("Fundamental matrix must be a 3x3 matrix");
      return;
    }
    cv::Mat Ain(Intrinsics.clone());
    if (Ain.empty())
    {
      // Allocate a plain identity matrix for the camera matrix if the
      // Intrinsics
      ROS_ERROR("No camera parameters provided; using identity matrix instead"
                " -- This will probably result in a bad transformation.");
      Ain = cv::Mat::eye(Fin.rows, Fin.cols, CV_64FC1);
    }

    // std::stringstream Finstr;
    // std::stringstream Foutstr;
    // std::stringstream Ainstr;
    // std::stringstream Aoutstr;

    // Convert to LAPACK variables
    LaGenMatDouble F = LaGenMatDouble::zeros(Fin.rows, Fin.cols);
    LaGenMatDouble A = LaGenMatDouble::zeros(Fin.rows, Fin.cols);

//    Finstr << "cv::Mat Fundamental Matrix = \n";
//    Foutstr << "LAPACK Fundamental Matrix = \n";
//    Ainstr << "cv::Mat Camera Matrix = \n";
//    Aoutstr << "LAPACK Camera Matrix = \n";
    for (int i = 0; i < Fin.rows; i++)
    {
      for (int j = 0; j < Fin.cols; j++)
      {
        F(i, j) = Fin.at<double>(i, j);
        A(i, j) = Ain.at<double>(i, j);
//        Finstr << Fin.at<double>(i,j) << "  ";
//        Ainstr << Ain.at<double>(i,j) << "  ";
//        Foutstr << F(i,j) << "  ";
//        Aoutstr << A(i,j) << "  ";
      }

//      Finstr << "\n\n";
//      Foutstr << "\n\n";
//      Ainstr << "\n\n";
//      Aoutstr << "\n\n";
    }

    // TODO(kkozak): Check to make sure that the matrices are the same after
    //               copying:
//    ROS_ERROR("Please check to make sure that the following in and out matrices"
//              " match, and if they do, delete the related print statements from"
//              " the code ");
//
//    fprintf(stderr,"%s",Finstr.str().c_str());
//    fprintf(stderr,"%s",Foutstr.str().c_str());
//    fprintf(stderr,"%s",Ainstr.str().c_str());
//    fprintf(stderr,"%s",Aoutstr.str().c_str());




    // Convert Fundamental matrix to the Essential matrix --> E = A'*F*A
    LaGenMatDouble Etemp(3, 3);
    LaGenMatDouble E(3, 3);
    Blas_Mat_Trans_Mat_Mult(A, F, Etemp);  // Etemp = A'*F
    Blas_Mat_Mat_Mult(Etemp, A, E);        // E = Etemp*A

    // Perform SVD:
    LaGenMatDouble U(3, 3);      // U
    LaVectorDouble Stemp(3, 1);  // S
    LaGenMatDouble Vt(3, 3);     // V'

    // Note that E gets modified in this step, so it can't be used after this
    LaSVD_IP(E, Stemp, U, Vt);

    if (std::abs(Stemp(0) - Stemp(1)) / Stemp(0) > 0.05)
    {
      ROS_ERROR("Singular values differ by more than 5%%. Transformation may be"
                " inaccurate.");
    }

    // Convert the singular value vector into a matrix
    LaGenMatDouble S(3, 3);
    S = LaGenMatDouble::from_diag(Stemp);

    ROS_ERROR("Singular Values Matrix");
    PrintMatrix(S);

    LaGenMatDouble W = LaGenMatDouble::zeros(3, 3);
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    LaGenMatDouble Z = LaGenMatDouble::zeros(3, 3);
    Z(0, 1) = -1;
    Z(1, 0) = 1;


    // Compute the Rotation Matrix
    LaGenMatDouble Rtemp1(3, 3);
    LaGenMatDouble Rtemp2(3, 3);
    // Rtemp1 = U*W^-1 = U*W'
    Blas_Mat_Mat_Trans_Mult(U, W, Rtemp1);
    // Rtemp2 = Rtemp1*V' = Rtemp1*Vt = U*W'*Vt
    Blas_Mat_Mat_Mult(Rtemp1, Vt, Rtemp2);

    // Compute the Rotation Matrix again using a different solution
    LaGenMatDouble Rtemp3(3, 3);
    LaGenMatDouble Rtemp4(3, 3);
    // Rtemp3 = U*W
    Blas_Mat_Mat_Mult(U, W, Rtemp3);
    // Rtemp4 = Rtemp1*V' = Rtemp1*Vt = U*W'*Vt
    Blas_Mat_Mat_Mult(Rtemp3, Vt, Rtemp4);


    // Compute the Translation Vector
    LaGenMatDouble Ttemp1(3, 3);
    LaGenMatDouble Ttemp2(3, 3);
    // Ttemp1 = Vt'*Z = V*Z -- Using Z which better deals with the case where
    // sigma1 != sigma2
    Blas_Mat_Trans_Mat_Mult(Vt, Z, Ttemp1);
    // Ttemp2 = Ttemp1*Vt = Ttemp1*V' = V*Z*V'
    Blas_Mat_Mat_Mult(Ttemp1, Vt, Ttemp2);

//    Blas_Mat_Trans_Mat_Mult(Vt,W,Ttemp1); // Ttemp1 = Vt'*W = V*W
//    Blas_Mat_Mat_Mult(Ttemp1,S,Ttemp2);   // Ttemp2 = Ttemp1*S
//    Blas_Mat_Mat_Mult(Ttemp2,Vt,Ttemp1);  // Ttemp1 = Ttemp2*Vt

    // copy the elements to the output vectors:
    // TODO(kkozak): Check to make sure this approach (to allocation and such)
    //               makes sense
    R1 = cv::Mat::zeros(3, 3, CV_32F);
    R2 = cv::Mat::zeros(3, 3, CV_32F);
    T = cv::Mat::zeros(3, 1, CV_32F);

    for (uint32_t i = 0; i < 3; i++)
    {
      for (uint32_t j = 0; j < 3; j++)
      {
        R1.at<float>(i, j) = Rtemp2(i, j);
        R2.at<float>(i, j) = Rtemp4(i, j);
      }
    }

    // We need to get the vector elements from the skew symmetric matrix
//    T.at<float>(0,0) = Ttemp1(2,1);
//    T.at<float>(1,0) = Ttemp1(0,2);
//    T.at<float>(2,0) = Ttemp1(1,0);
    T.at<float>(0, 0) = Ttemp2(2, 1);
    T.at<float>(1, 0) = Ttemp2(0, 2);
    T.at<float>(2, 0) = Ttemp2(1, 0);


    ROS_ERROR("Translation Matrix");
    PrintMatrix(Ttemp2);
    ROS_ERROR("Rotation Matrix");
    PrintMatrix(Rtemp2);
    ROS_ERROR("Rotation Matrix 2 = ");
    PrintMatrix(Rtemp4);
  }

  cv::Mat ComputeRigid2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2,
      cv::Mat& inliers1,
      cv::Mat& inliers2,
      std::vector<uint32_t> &good_points)
  {
    cv::Mat T;
    // Here we are trying to compute the transformation matrix T, where T most
    // closely satisfies the relationship
    // T*[points1(i);1] = [points2(i);1] for inlying points in both sets
    // We use RANSAC to exclude outliers

    // First build the input vectors
    bool row_order = false;
    if (!ValidPointsForTransform(points1, points2))
    {
      ROS_ERROR("Invalid points for calculating transform.");
      return T;
    }

    uint32_t NumPoints = points1.cols;
    if (points1.rows > 1)
    {
      row_order = true;
      NumPoints = points1.rows;
    }

    // Setup matrices for holding all of the points
    LaGenMatDouble sourceA(NumPoints, 2);
    LaGenMatDouble sourceA_aug(NumPoints, 3);
    LaGenMatDouble sourceB(NumPoints, 2);
    LaGenMatDouble tempA(NumPoints, 2);

    double xmax = -1.0e20;
    double xmin = 1.0e20;
    double ymax = xmax;
    double ymin = xmin;

    // Convert from cv::Mat to LaGenMatDouble
    for (uint32_t i = 0; i < NumPoints; i++)
    {
      // Points from first frame
      if (row_order)
      {
        sourceA(i, 0) = points1.at<cv::Vec2f>(i, 0)[0];
        sourceA(i, 1) = points1.at<cv::Vec2f>(i, 0)[1];
        sourceB(i, 0) = points2.at<cv::Vec2f>(i, 0)[0];
        sourceB(i, 1) = points2.at<cv::Vec2f>(i, 0)[1];
        sourceA_aug(i, 0) = points1.at<cv::Vec2f>(i, 0)[0];
        sourceA_aug(i, 1) = points1.at<cv::Vec2f>(i, 0)[1];
      }
      else
      {
        sourceA(i, 0) = points1.at<cv::Vec2f>(0, i)[0];
        sourceA(i, 1) = points1.at<cv::Vec2f>(0, i)[1];
        sourceB(i, 0) = points2.at<cv::Vec2f>(0, i)[0];
        sourceB(i, 1) = points2.at<cv::Vec2f>(0, i)[1];
        sourceA_aug(i, 0) = points1.at<cv::Vec2f>(0, i)[0];
        sourceA_aug(i, 1) = points1.at<cv::Vec2f>(0, i)[1];
      }
      sourceA_aug(i, 2) = 1.0;

      // Find the range of the data;
      if (sourceA(i, 0) > xmax)
      {
        xmax = sourceA(i, 0);
      }
      else if (sourceA(i, 0) < xmin)
      {
        xmin = sourceA(i, 0);
      }

      if (sourceA(i, 1) > ymax)
      {
        ymax = sourceA(i, 1);
      }
      else if (sourceA(i, 1) < ymin)
      {
        ymin = sourceA(i, 1);
      }
    }

    // Specify RANSAC Parameters:
    uint32_t NumberOfPointsToSample = 3;
    uint32_t MaxNumberOfIterations = 4000;
    uint32_t MinNumValidPointsNeeded = 6;
    double   EscapeLevel = 0.8;
    // Perhaps change this to be a fraction of max range or make it a parameter
    double   MaxReprojError = 20;
    double   maxRNorm = 0.000000000000001;


    // Setup the matrices for doing the sample estimates of the Transformation
    // Matrix
    LaGenMatDouble A(NumberOfPointsToSample, 3);
    LaGenMatDouble B(NumberOfPointsToSample, 2);
    LaGenMatDouble X(3, 2);
    LaGenMatDouble X_good(3, 2);


    // Do RANSAC
    for (uint32_t i = 0; i < MaxNumberOfIterations; ++i)
    {
      std::vector<uint32_t> p1;

      // Generate a random set of indices
      RandPermSet(NumPoints, NumberOfPointsToSample, p1);

      // Fill the matrices used to solve for the Sample Transformation Matrix, X
      for (uint32_t j = 0; j < p1.size(); j++)
      {
        A(j, 0) = sourceA(p1[j], 0);
        A(j, 1) = sourceA(p1[j], 1);
        A(j, 2) = 1.0;

        B(j, 0) = sourceB(p1[j], 0);
        B(j, 1) = sourceB(p1[j], 1);
      }

      // Solve for the Transformation matrix, X
      try
      {
        LaLinearSolve(A, X, B);
      }
      catch (const std::exception& e)
      {
        // ROS_INFO("Got LaLinearSolve Error ignore OK: %s", e.what());
        // when multiple potential matches are allowed, this has the potential
        // to be exactly singular
        continue;
      }

      double cn;  // condition number
      double rnorm;
      RegularizeTransform(X, cn, rnorm);

      // Check to see whether the rotation matrix is close to valid
      // TODO(kkozak): Parameterize this
      if (fabs(cn - 1.0) > .1 || rnorm > maxRNorm)
      {
        // ROS_ERROR("Did not meet condition number or rnorm criteria: cn = %f,
        // rnorm = %g",cn,rnorm);
        continue;
      }

      // tempA is a list vectors between projected points and actual points
      tempA = sourceB;
      // The following implements: tempA = sourceA_aug * X - sourceB;
      Blas_Mat_Mat_Mult(sourceA_aug, X, tempA, false, false, 1.0, -1.0);

      // Find all of the points within the re-projection error bound (add their
      // indices to temp_points)
      std::vector<uint32_t> temp_points;
      for (int j = 0; j < tempA.rows(); j++)
      {
        double dist = sqrt(pow(tempA(j, 0), 2) + pow(tempA(j, 1), 2));

        if (dist < MaxReprojError)
        {
          temp_points.push_back(j);
        }
      }

      if (temp_points.size() > good_points.size())
      {
        good_points = temp_points;
      }

      if (good_points.size() >= MinNumValidPointsNeeded
          && good_points.size() >= static_cast<uint32_t>(EscapeLevel*static_cast<double>(NumPoints)))
      {
        break;
        // We've met the escape criteria, so we don't need to keep iterating
      }
    }  // end of 100 iterations


    if (good_points.size() >= MinNumValidPointsNeeded)
    {
      // Compute a final transform using all the valid points.
      if (row_order)
      {
        inliers1 = cv::Mat(good_points.size(), 1, CV_32FC2);
        inliers2 = cv::Mat(good_points.size(), 1, CV_32FC2);
      }
      else
      {
        inliers1 = cv::Mat(1, good_points.size(), CV_32FC2);
        inliers2 = cv::Mat(1, good_points.size(), CV_32FC2);
      }

      for (uint32_t i = 0; i < good_points.size(); ++i)
      {
        if (row_order)
        {
          inliers1.at<cv::Vec2f>(i, 0) =
              cv::Vec2f(sourceA(good_points[i], 0), sourceA(good_points[i], 1));
          inliers2.at<cv::Vec2f>(i, 0) =
              cv::Vec2f(sourceB(good_points[i], 0), sourceB(good_points[i], 1));
        }
        else
        {
          inliers1.at<cv::Vec2f>(0, i) =
              cv::Vec2f(sourceA(good_points[i], 0), sourceA(good_points[i], 1));
          inliers2.at<cv::Vec2f>(0, i) =
              cv::Vec2f(sourceB(good_points[i], 0), sourceB(good_points[i], 1));
        }
      }

      T = LeastSqauresRigid2DTransform(inliers1, inliers2);
    }
    else
    {
      ROS_DEBUG("Not enough valid points size of good_points =%d",
                (int)good_points.size());
    }

    return T;
  }

  cv::Mat ComputeLooseAffine2DTransform(
      const cv::Mat& points1,
      const cv::Mat& points2,
      cv::Mat& inliers1,
      cv::Mat& inliers2,
      cv::Mat& T_rigid,
      double& rms_error)
  {
    cv::Mat Affine;
    std::vector<uint32_t> good_points;
    // Here we are trying to compute the affine transformation matrix Affine,
    // where A most closely satisfies the relationship
    // T*[points1(i);1] = [points2(i);1] for inlying points in both sets
    // We use RANSAC to exclude outliers

    // First build the input vectors
    bool row_order = true;
    if (!ValidPointsForTransform(points1, points2))
    {
      ROS_ERROR("Invalid points for calculating transform.");
      return Affine;
    }

    uint32_t NumPoints = points1.cols;
    if (points1.rows > 1)
    {
      row_order = false;
      NumPoints = points1.rows;
    }

    // Setup matrices for holding all of the points
    LaGenMatDouble sourceA(NumPoints, 2);
    LaGenMatDouble sourceA_aug(NumPoints, 3);
    LaGenMatDouble sourceB(NumPoints, 2);
    LaGenMatDouble tempA(NumPoints, 2);

    double xmax = -1.0e20;
    double xmin = 1.0e20;
    double ymax = xmax;
    double ymin = xmin;

    // Convert from cv::Mat to LaGenMatDouble
    for (uint32_t i = 0; i < NumPoints; i++)
    {
      // Points from first frame
      if (row_order)
      {
        sourceA(i, 0) = points1.at<cv::Vec2f>(0, i)[0];
        sourceA(i, 1) = points1.at<cv::Vec2f>(0, i)[1];
        sourceB(i, 0) = points2.at<cv::Vec2f>(0, i)[0];
        sourceB(i, 1) = points2.at<cv::Vec2f>(0, i)[1];
        sourceA_aug(i, 0) = points1.at<cv::Vec2f>(0, i)[0];
        sourceA_aug(i, 1) = points1.at<cv::Vec2f>(0, i)[1];
      }
      else
      {
        sourceA(i, 0) = points1.at<cv::Vec2f>(i, 0)[0];
        sourceA(i, 1) = points1.at<cv::Vec2f>(i, 0)[1];
        sourceB(i, 0) = points2.at<cv::Vec2f>(i, 0)[0];
        sourceB(i, 1) = points2.at<cv::Vec2f>(i, 0)[1];
        sourceA_aug(i, 0) = points1.at<cv::Vec2f>(i, 0)[0];
        sourceA_aug(i, 1) = points1.at<cv::Vec2f>(i, 0)[1];
      }
      sourceA_aug(i, 2) = 1.0;

      // Find the range of the data;
      if (sourceA(i, 0) > xmax)
      {
        xmax = sourceA(i, 0);
      }
      else if (sourceA(i, 0) < xmin)
      {
        xmin = sourceA(i, 0);
      }

      if (sourceA(i, 1) > ymax)
      {
        ymax = sourceA(i, 1);
      }
      else if (sourceA(i, 1) < ymin)
      {
        ymin = sourceA(i, 1);
      }
    }

    // Specify RANSAC Parameters:
    uint32_t NumberOfPointsToSample = 3;
    uint32_t MaxNumberOfIterations = 100;
    uint32_t MinNumValidPointsNeeded = 6;
    double   EscapeLevel = 0.8;
    // Perhaps change this to be a fraction of max range or make it a parameter
    double   MaxReprojError = 30;  // Use a looser reprojection error to capture
                                   // inliers in cases where there is strong
                                   // perspective distortion
    double   maxRNorm = 0.000000000000001;


    // Setup the matrices for doing the sample estimates of the Transformation
    // Matrix
    LaGenMatDouble A(NumberOfPointsToSample, 3);
    LaGenMatDouble B(NumberOfPointsToSample, 2);
    LaGenMatDouble X(3, 2);
    LaGenMatDouble X_good(3, 2);

    // Do RANSAC
    for (uint32_t i = 0; i < MaxNumberOfIterations; ++i)
    {
      std::vector<uint32_t> p1;

      // Generate a random set of indices
      RandPermSet(NumPoints, NumberOfPointsToSample, p1);

      // Fill the matrices used to solve for the Sample Transformation Matrix, X
      for (uint32_t j = 0; j < p1.size(); j++)
      {
        A(j, 0) = sourceA(p1[j], 0);
        A(j, 1) = sourceA(p1[j], 1);
        A(j, 2) = 1.0;

        B(j, 0) = sourceB(p1[j], 0);
        B(j, 1) = sourceB(p1[j], 1);
      }

      // Solve for the Transformation matrix, X
      try
      {
        LaLinearSolve(A, X, B);
      }
      catch (const std::exception& e)
      {
        // when multiple potential matches are allowed, this has the potential
        // to be exactly singular, and when it is, LAPACK may/will throw an
        // exception.  In those cases, the computed transform would obviously
        // be no good so just continue;
        continue;
      }

      double cn;  // condition number
      double rnorm;
      RegularizeTransform(X, cn, rnorm);

      // Check to see whether the rotation matrix is close to valid
      // TODO(kkozak): Parameterize this
      if (fabs(cn - 1.0) > .1 || rnorm > maxRNorm)
      {
        // ROS_ERROR("Did not meet condition number or rnorm criteria: cn = %f,
        // rnorm = %g",cn,rnorm);
        continue;
      }

      // tempA is a list vectors between projected points and actual points
      tempA = sourceB;
      //  The following implements: tempA = sourceA_aug * X - sourceB;
      Blas_Mat_Mat_Mult(sourceA_aug, X, tempA, false, false, 1.0, -1.0);

      // Find all of the points within the re-projection error bound (add their
      // indices to temp_points)
      std::vector<uint32_t> temp_points;
      for (int j = 0; j < tempA.rows(); j++)
      {
        double dist = sqrt(pow(tempA(j, 0), 2) + pow(tempA(j, 1), 2));

        if (dist < MaxReprojError)
        {
          temp_points.push_back(j);
        }
      }

      if (temp_points.size() > good_points.size())
      {
        good_points = temp_points;
      }

      if (good_points.size() >= MinNumValidPointsNeeded
          && good_points.size() >= static_cast<uint32_t>(EscapeLevel * static_cast<double>(NumPoints)))
      {
        break;
        // We've met the escape criteria, so we don't need to keep iterating
      }
    }  // end of 100 iterations


    if (good_points.size() >= MinNumValidPointsNeeded)
    {
      if (row_order)
      {
        inliers1 = cv::Mat(cv::Size(good_points.size(), 1), CV_32FC2);
        inliers2 = cv::Mat(cv::Size(good_points.size(), 1), CV_32FC2);
      }
      else
      {
        inliers1 = cv::Mat(cv::Size(1, good_points.size()), CV_32FC2);
        inliers2 = cv::Mat(cv::Size(1, good_points.size()), CV_32FC2);
      }

      // Compute a final transform using all the valid points
      LaGenMatDouble A1(good_points.size(), 3);
      LaGenMatDouble B1(good_points.size(), 2);
      for (uint32_t i = 0; i < good_points.size(); ++i)
      {
        A1(i, 0) = sourceA(good_points[i], 0);
        A1(i, 1) = sourceA(good_points[i], 1);
        A1(i, 2) = 1.0;

        B1(i, 0) = sourceB(good_points[i], 0);
        B1(i, 1) = sourceB(good_points[i], 1);

        if (row_order)
        {
          inliers1.at<cv::Vec2f>(i, 0) =
              cv::Vec2f(sourceA(good_points[i], 0), sourceA(good_points[i], 1));
          inliers2.at<cv::Vec2f>(i, 0) =
              cv::Vec2f(sourceB(good_points[i], 0), sourceB(good_points[i], 1));
        }
        else
        {
          inliers1.at<cv::Vec2f>(0, i) =
              cv::Vec2f(sourceA(good_points[i], 0), sourceA(good_points[i], 1));
          inliers2.at<cv::Vec2f>(0, i) =
              cv::Vec2f(sourceB(good_points[i], 0), sourceB(good_points[i], 1));
        }
      }
      // Solve for the Transformation matrix, X
      LaLinearSolve(A1, X, B1);

      double dx_sum = 0.0;
      double dy_sum = 0.0;

      // Fix the translation vector to correspond to the difference between
      // means (centroids)
      for (int32_t i = 0; i < A1.rows(); ++i)
      {
        dx_sum += B1(i, 0) - A1(i, 0);
        dy_sum += B1(i, 1) - A1(i, 1);
      }
      X(2, 0) = dx_sum / A1.rows();
      X(2, 1) = dy_sum / A1.rows();

      // Compute the Mean Squared Error
      // tempA is a list vectors between projected points and actual points
      tempA = B1;
      // The following implements: tempA = sourceA_aug * X - sourceB;
      LaGenMatDouble X_short_temp = X;
      double cn1;
      double rnorm1;

      RegularizeTransform(X_short_temp, cn1, rnorm1);

      Blas_Mat_Mat_Mult(A1, X_short_temp, tempA, false, false, 1.0, -1.0);

      std::vector<double> dist_err;

      for (int32_t i = 0; i < A1.rows(); ++i)
      {
        double temp_dist = std::sqrt(tempA(i, 0) * tempA(i, 0)
                                     + tempA(i, 1) * tempA(i, 1));

        dist_err.push_back(temp_dist);
      }

      rms_error = 0.0;
      std::sort(dist_err.begin(), dist_err.end());
      int n = dist_err.size();
      for (int i = 0; i < n; ++i)
      {
        rms_error += dist_err[i];
      }
      std::sort(dist_err.begin(), dist_err.end());
      int mid_idx = dist_err.size() / 2;
      rms_error = dist_err[mid_idx];

      double cn;
      double rnorm;

      Affine.create(2, 3, CV_32FC1);
      for (uint32_t i = 0; i < 2; i++)
      {
        for (uint32_t j = 0; j < 3; j++)
        {
          Affine.at<float>(i, j) = X(j, i);
        }
      }

      RegularizeTransform(X, cn, rnorm);
      T_rigid.release();
      T_rigid.create(2, 3, CV_32FC1);

      for (uint32_t i = 0; i < 2; i++)
      {
        for (uint32_t j = 0; j < 3; j++)
        {
          T_rigid.at<float>(i, j) = X(j, i);
        }
      }
    }
    else
    {
      ROS_DEBUG("Not enough valid points size of good_points =%d",
                (int)good_points.size());
    }
    return Affine;
  }

  cv::Mat ComputeRigid3DTransform(cv::Mat& points1, cv::Mat& points2)
  {
    cv::Mat T;
    // Here we are trying to compute the transformation matrix T, where T most
    // closely satisfies the relationship
    // T*[points1(i);1] = [points2(i);1] for inlying points in both sets
    // We use RANSAC to exclude outliers

    // First build the input vectors
    if (points1.cols != points2.cols || points1.cols  < points1.rows)
    {
      ROS_ERROR("Input to computeRigid3DTransform incorrect.  Be sure that"
                " the points are interlaced in a single row for each array.");
      return T;
    }
    if ((points1.type() != CV_32FC2 && points1.type() != CV_32FC3)
        || (points2.type() != CV_32FC2 && points2.type() != CV_32FC3))
    {
      ROS_ERROR("Input Mat type must be either CV_32FC2 (for 2D) or CV_32FC3 "
                "(for 3D)");
      return T;
    }

    uint32_t NumPoints = points1.cols;

    // Setup matrices for holding all of the points
    LaGenMatDouble sourceA(NumPoints, 3);
    LaGenMatDouble sourceA_aug(NumPoints, 4);
    LaGenMatDouble sourceB(NumPoints, 3);
    LaGenMatDouble tempA(NumPoints, 3);

    double xmax = -1.0e20;
    double xmin = 1.0e20;
    double ymax = xmax;
    double ymin = xmin;

    // Convert from cv::Mat to LaGenMatDouble
    for (uint32_t i = 0; i < NumPoints; i++)
    {
      // Points from first frame
      sourceA(i, 0) = points1.at<cv::Vec3f>(0, i)[0];
      sourceA(i, 1) = points1.at<cv::Vec3f>(0, i)[1];
      sourceA(i, 2) = points1.at<cv::Vec3f>(0, i)[2];

      // Find the range of the data;
      if (sourceA(i, 0) > xmax)
      {
        xmax = sourceA(i, 0);
      }
      else if (sourceA(i, 0) < xmin)
      {
        xmin = sourceA(i, 0);
      }

      if (sourceA(i, 1) > ymax)
      {
        ymax = sourceA(i, 1);
      }
      else if (sourceA(i, 1) < ymin)
      {
        ymin = sourceA(i, 1);
      }

      // setup augmented vectors
      sourceA_aug(i, 0) = points1.at<cv::Vec3f>(0, i)[0];
      sourceA_aug(i, 1) = points1.at<cv::Vec3f>(0, i)[1];
      sourceA_aug(i, 2) = points1.at<cv::Vec3f>(0, i)[2];
      sourceA_aug(i, 3) = 1.0;

      sourceB(i, 0) = points2.at<cv::Vec3f>(0, i)[0];
      sourceB(i, 1) = points2.at<cv::Vec3f>(0, i)[1];
      sourceB(i, 2) = points2.at<cv::Vec3f>(0, i)[2];
    }

    // Specify RANSAC Parameters:
    uint32_t NumberOfPointsToSample = 6;
    uint32_t MaxNumberOfIterations = 100;
    uint32_t MinNumValidPointsNeeded = 8;
    double   EscapeLevel = 0.8;
    // Perhaps change this to be a fraction of max range or make it a parameter
    double   MaxReprojError = 1.0;
    double   maxRNorm = 0.000000000000001;
    std::vector<uint32_t> good_points;

    // Setup the matrices for doing the sample estimates of the Transformation
    // Matrix
    LaGenMatDouble A(NumberOfPointsToSample, 4);
    LaGenMatDouble B(NumberOfPointsToSample, 3);
    LaGenMatDouble X(4, 3);
    LaGenMatDouble X_good(4, 3);

    // Do RANSAC
    for (uint32_t i = 0; i < MaxNumberOfIterations; ++i)
    {
      std::vector<uint32_t> p1;

      // Generate a random set of indices
      RandPermSet(NumPoints, NumberOfPointsToSample, p1);

      // Fill the matrices used to solve for the Sample Transformation Matrix, X
      for (uint32_t j = 0; j < p1.size(); j++)
      {
        A(j, 0) = sourceA(p1[j], 0);
        A(j, 1) = sourceA(p1[j], 1);
        A(j, 2) = sourceA(p1[j], 2);
        A(j, 3) = 1.0;

        B(j, 0) = sourceB(p1[j], 0);
        B(j, 1) = sourceB(p1[j], 1);
        B(j, 2) = sourceB(p1[j], 2);
      }

      // Solve for the Transformation matrix, X
      LaLinearSolve(A, X, B);

      double cn;
      double rnorm;
      RegularizeTransform(X, cn, rnorm);

      // Check to see whether the rotation matrix is close to valid
      // TODO(kkozak): Parameterize this
      if (fabs(cn - 1.0) > 0.1 || rnorm > maxRNorm)
      {
        // ROS_ERROR("Did not meet condition number or rnorm criteria: "
        // "cn = %f, rnorm = %f",cn,rnorm);
        continue;
      }

      // tempA is a list vectors between projected points and actual points
      tempA = sourceB;
      // The following implements: tempA = sourceA_aug * X - sourceB;
      Blas_Mat_Mat_Mult(sourceA_aug, X, tempA, false, false, 1.0, -1.0);


      // Find all of the points within the re-projection error bound (add their
      // indices to temp_points)
      std::vector<uint32_t> temp_points;
      for (int j = 0; j < tempA.rows(); j++)
      {
        double dist = sqrt(pow(tempA(j, 0), 2) + pow(tempA(j, 1), 2) +
                           pow(tempA(j, 2), 2));

        if (dist < MaxReprojError)
        {
          temp_points.push_back(j);
        }
      }

      if (temp_points.size() > good_points.size())
      {
        good_points = temp_points;
      }

      if (good_points.size() >= MinNumValidPointsNeeded
          && good_points.size() >= static_cast<uint32_t>(EscapeLevel*static_cast<double>(NumPoints)))
      {
        break;
        // We've met the escape criteria, so we don't need to keep iterating
      }
    }

    if (good_points.size() >= MinNumValidPointsNeeded)
    {
      // We have a valid set, so let's compute a final transform using all the
      // valid points
      LaGenMatDouble A1(good_points.size(), 4);
      LaGenMatDouble B1(good_points.size(), 3);
      for (uint32_t i = 0; i < good_points.size(); ++i)
      {
        A1(i, 0) = sourceA(good_points[i], 0);
        A1(i, 1) = sourceA(good_points[i], 1);
        A1(i, 2) = sourceA(good_points[i], 2);
        A1(i, 3) = 1.0;

        B1(i, 0) = sourceB(good_points[i], 0);
        B1(i, 1) = sourceB(good_points[i], 1);
        B1(i, 2) = sourceB(good_points[i], 2);
      }

      // Solve for the Transformation matrix, X
      LaLinearSolve(A1, X, B1);
      double cn;
      double rnorm;
      RegularizeTransform(X, cn, rnorm);

      T.create(3, 4, CV_32FC1);
      for (uint32_t i = 0; i < 3; i++)
      {
        for (uint32_t j = 0; j < 4; j++)
        {
          T.at<float>(i, j) = X(j, i);
        }
      }
    }

    return T;
  }

  void RegularizeTransform(
      LaGenMatDouble &T,
      double &conditionNum,
      double &rnorm)
  {
    // Note that T here is really T' without the extra column ==> [R';p];
    LaGenMatDouble rot(T.cols(), T.cols());
    GetR(T, rot);
    RegularizeRotation(rot, conditionNum, rnorm);
    for (int i = 0; i < T.cols(); i++)
    {
      for (int j = 0; j < T.cols(); j++)
      {
        T(i, j) =  rot(i, j);
      }
    }
  }

  void GetR(const LaGenMatDouble& T, LaGenMatDouble& R)
  {
    // Note that T here is really T' without the extra column ==> [R';p];
    for (int i = 0; i < T.cols(); i++)
    {
      for (int j = 0; j < T.cols(); j++)
      {
        R(i, j) = T(i, j);
      }
    }
  }

  void RegularizeRotation(
      LaGenMatDouble &rot,
      double &conditionNum,
      double &rnorm)
  {
    uint32_t N = rot.cols();
    LaVectorDouble Sigma(N);
    LaGenMatDouble U(N, N);
    LaGenMatDouble Vt(N, N);
    LaGenMatDouble RtR(N, N);

    LaSVD_IP(rot, Sigma, U, Vt);
    conditionNum = Sigma(0) / Sigma(N-1);

    // find the diagonal matrix S whose values are either 1, when scaling is
    // not allowed, or the average singular value when it is
    double diagonal = 0;
    for (uint32_t i = 0; i < N; i++)
    {
      diagonal += Sigma(i);
    }

    // set diagonal to 1
    diagonal = 1.0;

    // compute the diagonal matrix
    LaGenMatDouble S(N, N);
    for (uint32_t i = 0; i < N; i++)
    {
      for (uint32_t j = 0; j < N; j++)
      {
        if (i == j)
        {
          S(i, j) = diagonal;
        }
        else
        {
          S(i, j) = 0;
        }
      }
    }

    // Compute the rotation as USVt
    rot = U * S * Vt;

    // compute the rotation measure as ||RtR - StS||
    Blas_Mat_Trans_Mat_Mult(rot, rot, RtR);
    rnorm = Blas_NormF(RtR - S * S);
  }

  void RandPermSet(
      uint32_t max_num,
      uint32_t total_samples,
      std::vector<uint32_t>& indices)
  {
    indices.clear();

    if (total_samples > max_num)
    {
      ROS_ERROR("Total samples is greater than max number. %d > %d",
                (int)total_samples,
                (int)max_num);
      return;
    }

    while (indices.size() < total_samples)
    {
      uint32_t sample = static_cast<uint32_t>(rand() % static_cast<int>(max_num));
      bool found = false;
      for (uint32_t i = 0; i < indices.size(); i++)
      {
        if (sample == indices[i])
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        indices.push_back(sample);
      }
    }
    return;
  }
}
