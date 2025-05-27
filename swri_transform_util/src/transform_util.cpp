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

#include <swri_transform_util/transform_util.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <swri_transform_util/earth_constants.h>
#include <swri_transform_util/local_xy_util.h>
#include <swri_math_util/constants.h>
#include <swri_math_util/math_util.h>
#include <swri_math_util/trig_util.h>

namespace swri_transform_util
{
  bool compare_rows(
      const std::pair<int32_t, double>& i,
      const std::pair<int32_t, double>& j)
  {
    return i.second > j.second;
  }

  tf2::Transform GetRelativeTransform(
        double latitude,
        double longitude,
        double yaw,
        double reference_latitude,
        double reference_longitude,
        double reference_yaw)
  {
    tf2::Transform transform = tf2::Transform::getIdentity();

    tf2::Quaternion reference_rotation = tf2::Quaternion::getIdentity();
    reference_rotation.setRotation(tf2::Vector3(0, 0, 1), reference_yaw);

    tf2::Quaternion rotation = tf2::Quaternion::getIdentity();
    rotation.setRotation(tf2::Vector3(0, 0, 1), yaw);

    transform.setRotation(reference_rotation.inverse() * rotation);

    double x, y;
    LocalXyFromWgs84(
        latitude, longitude,
        reference_latitude, reference_longitude,
        x, y);

    tf2::Vector3 origin =
        tf2::Transform(reference_rotation.inverse()) * tf2::Vector3(x, y, 0);
    transform.setOrigin(origin);

    return transform;
  }

  double GreatCircleDistance(
      double src_latitude,
      double src_longitude,
      double dst_latitude,
      double dst_longitude)
  {
    double lat1 = src_latitude * swri_math_util::_deg_2_rad;
    double lon1 = src_longitude * swri_math_util::_deg_2_rad;

    double lat2 = dst_latitude * swri_math_util::_deg_2_rad;
    double lon2 = dst_longitude * swri_math_util::_deg_2_rad;

    double distance = 2.0 * std::asin(std::sqrt(
      std::pow(std::sin((lat1 - lat2) / 2.0), 2.0) +
      std::cos(lat1) * std::cos(lat2) *
      std::pow(std::sin((lon1 - lon2) / 2.0), 2.0)));

    return _earth_mean_radius * distance;
  }

  double GreatCircleDistance(const tf2::Vector3& src, const tf2::Vector3& dst)
  {
    return GreatCircleDistance(src.y(), src.x(), dst.y(), dst.x());
  }

  double GetBearing(
      double source_latitude,
      double source_longitude,
      double destination_latitude,
      double destination_longitude)
  {
    double lat1 = source_latitude * swri_math_util::_deg_2_rad;
    double lon1 = source_longitude * swri_math_util::_deg_2_rad;

    double lat2 = destination_latitude * swri_math_util::_deg_2_rad;
    double lon2 = destination_longitude * swri_math_util::_deg_2_rad;

    double d_lon = lon2 - lon1;

    double y = std::sin(d_lon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
        std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);

    return std::atan2(y, x) * swri_math_util::_rad_2_deg;
  }

  void GetMidpointLatLon(
      double latitude1,
      double longitude1,
      double latitude2,
      double longitude2,
      double& mid_latitude,
      double& mid_longitude)
  {
    double d_lon = (longitude2 - longitude1) * swri_math_util::_deg_2_rad;

    double lat1 = latitude1 * swri_math_util::_deg_2_rad;
    double lat2 = latitude2 * swri_math_util::_deg_2_rad;
    double lon1 = longitude1 * swri_math_util::_deg_2_rad;

    double x = std::cos(lat2) * std::cos(d_lon);
    double y = std::cos(lat2) * std::sin(d_lon);
    double tmp = std::cos(lat1) + x;
    double lat3 = std::atan2(
        std::sin(lat1) + std::sin(lat2), std::sqrt(tmp * tmp + y * y));
    double lon3 = lon1 + std::atan2(y, tmp);

    mid_latitude = lat3 * swri_math_util::_rad_2_deg;
    mid_longitude = lon3 * swri_math_util::_rad_2_deg;
  }

  double GetHeading(double src_x, double src_y, double dst_x, double dst_y)
  {
    return ToHeading(std::atan2(dst_y - src_y, dst_x - src_x));
  }

  double ToHeading(double yaw)
  {
    return swri_math_util::ToDegrees(swri_math_util::_half_pi - yaw);
  }

  double ToYaw(double heading)
  {
    return swri_math_util::ToRadians(-(heading - 90.0));
  }

  tf2::Quaternion SnapToRightAngle(const tf2::Quaternion& rotation)
  {
    if (rotation == tf2::Quaternion::getIdentity())
    {
      return rotation;
    }

    tf2::Matrix3x3 matrix(rotation);

    // First determine the order to process the rows in.  Rows with the largest
    // absolute max values will be ordered first.
    std::vector<std::pair<int32_t, double> > process_order(3);
    for (int32_t i = 0; i < 3; i++)
    {
      process_order[i].first = i;

      tf2::Vector3 row = matrix.getRow(i).absolute();
      process_order[i].second = row[row.maxAxis()];
    }
    std::sort(process_order.begin(), process_order.end(), compare_rows);

    // Update the rotation matrix by operating on each row in the determined
    // order.  Each row will be aligned to its primary axis such that a single
    // element in each row will be either 1 or -1 and the rest will be 0.
    for (int32_t i = 0; i < 3; i++)
    {
      int32_t row_num = process_order[i].first;
      tf2::Vector3 row = GetPrimaryAxis(matrix.getRow(row_num));

      for (int32_t j = 0; j < 3; j++)
      {
        matrix[row_num][j] = row[j];

        if (row[j] != 0)
        {
          for (int32_t k = 0; k < 3; k++)
          {
            if (k != row_num)
            {
              matrix[k][j] = 0;
            }
          }
        }
      }
    }

    // Verify that the resulting matrix is a valid rotation.
    if (!IsRotation(matrix))
    {
       // If this fails return the identity matrix.
       return tf2::Quaternion::getIdentity();
    }

    tf2::Quaternion snapped_rotation;
    matrix.getRotation(snapped_rotation);

    return snapped_rotation;
  }

  tf2::Vector3 GetPrimaryAxis(const tf2::Vector3& vector)
  {
    tf2::Vector3 vector_out = vector;

    if (vector.length() > 0)
    {
      double max = 0;
      int index = 0;

      for (int32_t i = 0; i < 3; i++)
      {
        if (std::fabs(vector[i]) > max)
        {
          max = std::fabs(vector[i]);
          index = i;
        }
      }

      for (int32_t i = 0; i < 3; i++)
      {
        if (i == index)
        {
          vector_out[i] = (vector[i] == 0 ? 0 :
            (vector[i] < 0 ? -1 : 1));
        }
        else
        {
          vector_out[i] = 0;
        }
      }
    }

    return vector_out;
  }

  bool IsRotation(tf2::Matrix3x3 matrix)
  {
    // Check that determinant is near 1.
    if (!swri_math_util::IsNear(matrix.determinant(), 1, 0.00001))
    {
      return false;
    }

    // Check that the each row is a unit vector.
    for (int32_t i = 0; i < 3; i++)
    {
      if (!swri_math_util::IsNear(matrix.getRow(i).length(), 1, 0.00001))
      {
        return false;
      }
    }

    // Check that the each column is a unit vector.
    for (int32_t i = 0; i < 3; i++)
    {
      if (!swri_math_util::IsNear(matrix.getColumn(i).length(), 1, 0.00001))
      {
        return false;
      }
    }

    return true;
  }

  tf2::Matrix3x3 GetUpperLeft(const std::array<double, 36>& matrix)
  {
    tf2::Matrix3x3 sub_matrix;

    sub_matrix[0][0] = matrix[0];
    sub_matrix[0][1] = matrix[1];
    sub_matrix[0][2] = matrix[2];
    sub_matrix[1][0] = matrix[6];
    sub_matrix[1][1] = matrix[7];
    sub_matrix[1][2] = matrix[8];
    sub_matrix[2][0] = matrix[12];
    sub_matrix[2][1] = matrix[13];
    sub_matrix[2][2] = matrix[14];

    return sub_matrix;
  }

  tf2::Matrix3x3 GetLowerRight(const std::array<double, 36>& matrix)
  {
    tf2::Matrix3x3 sub_matrix;

    sub_matrix[0][0] = matrix[21];
    sub_matrix[0][1] = matrix[22];
    sub_matrix[0][2] = matrix[23];
    sub_matrix[1][0] = matrix[27];
    sub_matrix[1][1] = matrix[28];
    sub_matrix[1][2] = matrix[29];
    sub_matrix[2][0] = matrix[33];
    sub_matrix[2][1] = matrix[34];
    sub_matrix[2][2] = matrix[35];

    return sub_matrix;
  }

  tf2::Matrix3x3 Get3x3Cov(const std::array<double, 9>& matrix)
  {
    tf2::Matrix3x3 matrix_out;

    matrix_out[0][0] = matrix[0];
    matrix_out[0][1] = matrix[1];
    matrix_out[0][2] = matrix[2];
    matrix_out[1][0] = matrix[3];
    matrix_out[1][1] = matrix[4];
    matrix_out[1][2] = matrix[5];
    matrix_out[2][0] = matrix[6];
    matrix_out[2][1] = matrix[7];
    matrix_out[2][2] = matrix[8];

    return matrix_out;
  }

  void Set3x3Cov(
      const tf2::Matrix3x3& matrix_in,
      std::array<double, 9>& matrix_out)
  {
    matrix_out[0] = matrix_in[0][0];
    matrix_out[1] = matrix_in[0][1];
    matrix_out[2] = matrix_in[0][2];
    matrix_out[3] = matrix_in[1][0];
    matrix_out[4] = matrix_in[1][1];
    matrix_out[5] = matrix_in[1][2];
    matrix_out[6] = matrix_in[2][0];
    matrix_out[7] = matrix_in[2][1];
    matrix_out[8] = matrix_in[2][2];
  }

  void SetUpperLeft(
      const tf2::Matrix3x3& sub_matrix,
      std::array<double, 36>& matrix)
  {
    matrix[0] = sub_matrix[0][0];
    matrix[1] = sub_matrix[0][1];
    matrix[2] = sub_matrix[0][2];
    matrix[6] = sub_matrix[1][0];
    matrix[7] = sub_matrix[1][1];
    matrix[8] = sub_matrix[1][2];
    matrix[12] = sub_matrix[2][0];
    matrix[13] = sub_matrix[2][1];
    matrix[14] = sub_matrix[2][2];
  }

  void SetLowerRight(
      const tf2::Matrix3x3& sub_matrix,
      std::array<double, 36>& matrix)
  {
    matrix[21] = sub_matrix[0][0];
    matrix[22] = sub_matrix[0][1];
    matrix[23] = sub_matrix[0][2];
    matrix[27] = sub_matrix[1][0];
    matrix[28] = sub_matrix[1][1];
    matrix[29] = sub_matrix[1][2];
    matrix[33] = sub_matrix[2][0];
    matrix[34] = sub_matrix[2][1];
    matrix[35] = sub_matrix[2][2];
  }

  double LongitudeDegreesFromMeters(
    double latitude,
    double altitude,
    double arc_length)
  {
    return arc_length / ((altitude + _earth_equator_radius)
                         * std::cos(latitude * swri_math_util::_deg_2_rad)) * swri_math_util::_rad_2_deg;
  }

  double LatitudeDegreesFromMeters(
    double altitude,
    double arc_length)
  {
    return arc_length / (altitude + _earth_equator_radius) * swri_math_util::_rad_2_deg;
  }

  std::string NormalizeFrameId(const std::string& frame_id)
  {
    if (!frame_id.empty() && frame_id[0] == '/')
    {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  bool FrameIdsEqual(const std::string& frame1, const std::string& frame2)
  {
    return (frame1 == frame2) ||
           (NormalizeFrameId(frame1) == NormalizeFrameId(frame2));
  }
}
