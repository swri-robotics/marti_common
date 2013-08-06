// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#ifndef TRANSFORM_UTIL_TRANSFORM_UTIL_H_
#define TRANSFORM_UTIL_TRANSFORM_UTIL_H_

#include <boost/array.hpp>

#include <tf/transform_datatypes.h>

#include <transform_util/local_xy_util.h>

namespace transform_util
{
  tf::Transform GetRelativeTransform(
      double latitude,
      double longitude,
      double yaw,
      double reference_latitude,
      double reference_longitude,
      double reference_yaw);

  double GetBearing(
      double source_latitude,
      double source_longitude,
      double destination_latitude,
      double destination_longitude);

  /**
   * Calculates the heading in degrees from a source and destination point in
   * a north-oriented (+y = north, +x = east), ortho-rectified coordinate
   * system. The heading is positive clock-wise with 0 degrees at north.
   *
   * @param[in] src_x The source X coordinate.
   * @param[in] src_y The source Y coordinate.
   * @param[in] dst_x The destination X coordinate.
   * @param[in] dst_y The destination Y coordinate.
   *
   * @returns The heading in degrees along the vector between the source and
   * destination points.
   */
  double GetHeading(double src_x, double src_y, double dst_x, double dst_y);

  /**
   * Convert yaw to heading, where yaw is in radians, counter-clockwise, with 0
   * on the positive x-axis and heading is in degrees, clockwise, with 0 on the
   * positive y-axis.
   *
   * @param[in] yaw The yaw in radians.
   *
   * @return The heading in degrees.
   */
  double ToHeading(double yaw);

  /**
   * Convert heading to yaw, where yaw is in radians, counter-clockwise, with 0
   * on the positive x-axis and heading is in degrees, clockwise, with 0 on the
   * positive y-axis.
   *
   * @param[in] heading The heading in degrees.
   *
   * @return The yaw in radians.
   */
  double ToYaw(double heading);

  /**
   * Snaps a quaternion rotation to the closest right angle rotation.
   *
   * @param[in]  rotation  The input quaternion rotation.
   *
   * @returns The closest right angle rotation to the input rotation.
   */
  tf::Quaternion SnapToRightAngle(const tf::Quaternion& rotation);

  /**
   * Return an axis aligned unit vector that is nearest the provided vector.
   *
   * @param[in]  vector  The input vector.
   *
   * @returns The axis aligned unit vector.
   */
  tf::Vector3 GetPrimaryAxis(const tf::Vector3& vector);

  /**
   * Validate that a 3x3 matrix is a rotation.
   *
   * @param[in]  matrix  The matrix to validate.
   *
   * @returns True if the matrix is a valid rotation.
   */
  bool IsRotation(tf::Matrix3x3 matrix);

  /**
   * Gets the upper-left 3x3 sub-matrix of a 6x6 matrix.
   *
   * @param[in]  matrix  The 6x6 matrix.
   *
   * @returns The upper-left 3x3 sub-matrix.
   */
  tf::Matrix3x3 GetUpperLeft(const boost::array<double, 36>& matrix);

  /**
   * Gets the lower-right 3x3 sub-matrix of a 6x6 matrix.
   *
   * @param[in]  matrix  The 6x6 matrix.
   *
   * @returns The lower-right 3x3 sub-matrix.
   */
  tf::Matrix3x3 GetLowerRight(const boost::array<double, 36>& matrix);

  /**
   * Converts the 3x3 covariance matrices from Imu messages to a Matrix3x3
   *
   * @param[in]  matrix   The input matrix
   *
   * @retval     Returns the input matrix as a Matrix3x3 object
   */
  tf::Matrix3x3 Get3x3Cov(const boost::array<double, 9>& matrix);

  /**
   * Converts the Matrix3x3 matrix into a 9 element covariance matrix from Imu
   * messages
   *
   * @param[in]  matrix_in    The input matrix
   * @param[out] matrix_out   The output matrix
   *
   */
  void Set3x3Cov(const tf::Matrix3x3& matrix_in,
                          boost::array<double, 9>& matrix_out);
  /**
   * Sets the upper-left quadrant of a 6x6 matrix with the specified 3x3
   * sub-matrix
   *
   * @param[in]  sub_matrix  The 3x3 sub-matrix.
   * @param[out] matrix      The 6x6 matrix to modify.
   */
  void SetUpperLeft(
      const tf::Matrix3x3& sub_matrix,
      boost::array<double, 36>& matrix);

  /**
   * Sets the lower-right quadrant of a 6x6 matrix with the specified 3x3
   * sub-matrix
   *
   * @param[in]  sub_matrix  The 3x3 sub-matrix.
   * @param[out] matrix      The 6x6 matrix to modify.
   */
  void SetLowerRight(
      const tf::Matrix3x3& sub_matrix,
      boost::array<double, 36>& matrix);
}

#endif  // TRANSFORM_UTIL_TRANSFORM_UTIL_H_

