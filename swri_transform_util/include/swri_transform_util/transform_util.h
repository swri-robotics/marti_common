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

#ifndef TRANSFORM_UTIL_TRANSFORM_UTIL_H_
#define TRANSFORM_UTIL_TRANSFORM_UTIL_H_

#include <string>
#include <array>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace swri_transform_util
{
  tf2::Transform GetRelativeTransform(
      double latitude,
      double longitude,
      double yaw,
      double reference_latitude,
      double reference_longitude,
      double reference_yaw);

  /**
   * Calculates the great circle distance between two points.
   *
   * @param[in] src_latitude   Source latitude
   * @param[in] src_longitude  Source longitude
   * @param[in] dst_latitude   Destination latitude
   * @param[in] dst_longitude  Destination longitude
   *
   * @returns Distance in meters.
   */
  double GreatCircleDistance(
      double src_latitude,
      double src_longitude,
      double dst_latitude,
      double dst_longitude);

  /**
   * Calculates the great circle distance between two points.
   *
   * @param[in] src  Source(x = longitude, y = latitude, z ignored)
   * @param[in] dst  Destination(x = longitude, y = latitude, z ignored)
   *
   * @returns Distance in meters.
   */
  double GreatCircleDistance(const tf2::Vector3& src, const tf2::Vector3& dst);

  /**
   * Calculates the bearing between two points.
   *
   * @param[in] source_latitude       The latitude of the origin point in degrees
   * @param[in] source_longitude      The longitude of the origin point in degrees
   * @param[in] destination_latitude  The latitude of the destination point in degrees
   * @param[in] destination_longitude The longitude of the destination point in degrees
   * @return The bearing between the origin and destination in degrees ENU
   */
  double GetBearing(
      double source_latitude,
      double source_longitude,
      double destination_latitude,
      double destination_longitude);

  /**
   * Find the midpoint on the arc between two lat/lon points
   *
   * @param[in] latitude1      Endpoint 1 latitude in degrees
   * @param[in] longitude1     Endpoint 1 longitude in degrees
   * @param[in] latitude2      Endpoint 2 latitude in degrees
   * @param[in] longitude2     Endpoint 2 longitude in degrees
   * @param[out] mid_latitude  Midpoint latitude in degrees
   * @param[out] mid_longitude Midpoint longitude in degrees
   */
  void GetMidpointLatLon(
      double latitude1,
      double longitude1,
      double latitude2,
      double longitude2,
      double& mid_latitude,
      double& mid_longitude);

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
  tf2::Quaternion SnapToRightAngle(const tf2::Quaternion& rotation);

  /**
   * Return an axis aligned unit vector that is nearest the provided vector.
   *
   * @param[in]  vector  The input vector.
   *
   * @returns The axis aligned unit vector.
   */
  tf2::Vector3 GetPrimaryAxis(const tf2::Vector3& vector);

  /**
   * Validate that a 3x3 matrix is a rotation.
   *
   * @param[in]  matrix  The matrix to validate.
   *
   * @returns True if the matrix is a valid rotation.
   */
  bool IsRotation(tf2::Matrix3x3 matrix);

  /**
   * Gets the upper-left 3x3 sub-matrix of a 6x6 matrix.
   *
   * @param[in]  matrix  The 6x6 matrix.
   *
   * @returns The upper-left 3x3 sub-matrix.
   */
  tf2::Matrix3x3 GetUpperLeft(const std::array<double, 36>& matrix);

  /**
   * Gets the lower-right 3x3 sub-matrix of a 6x6 matrix.
   *
   * @param[in]  matrix  The 6x6 matrix.
   *
   * @returns The lower-right 3x3 sub-matrix.
   */
  tf2::Matrix3x3 GetLowerRight(const std::array<double, 36>& matrix);

  /**
   * Converts the 3x3 covariance matrices from Imu messages to a Matrix3x3
   *
   * @param[in]  matrix   The input matrix
   *
   * @retval     Returns the input matrix as a Matrix3x3 object
   */
  tf2::Matrix3x3 Get3x3Cov(const std::array<double, 9>& matrix);

  /**
   * Converts the Matrix3x3 matrix into a 9 element covariance matrix from Imu
   * messages
   *
   * @param[in]  matrix_in    The input matrix
   * @param[out] matrix_out   The output matrix
   *
   */
  void Set3x3Cov(const tf2::Matrix3x3& matrix_in,
                          std::array<double, 9>& matrix_out);
  /**
   * Sets the upper-left quadrant of a 6x6 matrix with the specified 3x3
   * sub-matrix
   *
   * @param[in]  sub_matrix  The 3x3 sub-matrix.
   * @param[out] matrix      The 6x6 matrix to modify.
   */
  void SetUpperLeft(
      const tf2::Matrix3x3& sub_matrix,
      std::array<double, 36>& matrix);

  /**
   * Sets the lower-right quadrant of a 6x6 matrix with the specified 3x3
   * sub-matrix
   *
   * @param[in]  sub_matrix  The 3x3 sub-matrix.
   * @param[out] matrix      The 6x6 matrix to modify.
   */
  void SetLowerRight(
      const tf2::Matrix3x3& sub_matrix,
      std::array<double, 36>& matrix);

  /**
   * Calculate the subtending angle of an arc (aligned with the
   * longitudinal coordinate axis at the specified latitude and
   * altitude) with the specied arc length.
   *
   * @param[in] latitude     The latitude of the arc.
   * @param[in] altitude     The radius of the arc.
   * @param[in] arc_length   The arc length in meters.
   * @returns The angle subtended by the arc (in degrees)
   */
  double LongitudeDegreesFromMeters(
    double latitude,
    double altitude,
    double arc_length);

  /**
   * Calculate the subtending angle of an arc (aligned with the
   * latitudinal coordinate axis at the specified altitude) with the
   * specied arc length.  This conversion is invariant to longitude.
   *
   * @param[in] altitude     The radius of the arc.
   * @param[in] arc_length   The arc length in meters.
   * @returns The angle subtended by the arc (in degrees)
   */
  double LatitudeDegreesFromMeters(
    double altitude,
    double arc_length);

  /**
   * Normalize a TF frame ID by assuming that frames with
   * relative paths are in the global namespace and adding
   * a leading slash.
   *
   * @param[in] frame_id  A TF frame ID, with or without a leading slash
   * @returns   frame_id with a leading slash if it did not have one previously
   */
  std::string NormalizeFrameId(const std::string& frame_id);

  /**
   * Compare two TF frame IDs, assuming that frames with
   * relative paths are in the global namespace. E.g.
   * "map" == "/map"
   * "map" == "map"
   * "//map" != "/map"
   * "/ns/map" == "ns/map"
   * "/ns/map" != "map"
   * "/ns/map" != "/map"
   *
   * @param[in] frame1 A TF frame ID, with or without a leading slash
   * @param[in[ frame2 A TF frame ID, with or without a leading slash
   * @return True if the frames match
   **/
  bool FrameIdsEqual(const std::string& frame1, const std::string& frame2);
}

#endif  // TRANSFORM_UTIL_TRANSFORM_UTIL_H_

