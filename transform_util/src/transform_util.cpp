// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
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

#include <transform_util/transform_util.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#include <boost/math/special_functions/sign.hpp>

#include <math_util/constants.h>
#include <math_util/math_util.h>

namespace transform_util
{
  bool compare_rows(
      const std::pair<int32_t, double>& i,
      const std::pair<int32_t, double>& j)
  {
    return i.second > j.second;
  }

  tf::Transform GetRelativeTransform(
        double latitude,
        double longitude,
        double yaw,
        double reference_latitude,
        double reference_longitude,
        double reference_yaw)
  {
    tf::Transform transform = tf::Transform::getIdentity();

    tf::Quaternion reference_rotation = tf::Quaternion::getIdentity();
    reference_rotation.setRotation(tf::Vector3(0, 0, 1), reference_yaw);

    tf::Quaternion rotation = tf::Quaternion::getIdentity();
    rotation.setRotation(tf::Vector3(0, 0, 1), yaw);

    transform.setRotation(reference_rotation.inverse() * rotation);

    double x, y;
    LocalXyFromWgs84(
        latitude, longitude,
        reference_latitude, reference_longitude,
        x, y);

    tf::Vector3 origin =
        tf::Transform(reference_rotation) * tf::Vector3(x, y, 0);
    transform.setOrigin(origin);

    return transform;
  }

  double GetBearing(
      double source_latitude,
      double source_longitude,
      double destination_latitude,
      double destination_longitude)
  {
    double lat1 = source_latitude * math_util::_deg_2_rad;
    double lon1 = source_longitude * math_util::_deg_2_rad;

    double lat2 = destination_latitude * math_util::_deg_2_rad;
    double lon2 = destination_longitude * math_util::_deg_2_rad;

    double d_lon = lon2 - lon1;

    double y = std::sin(d_lon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
        std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);

    return std::atan2(y, x) * math_util::_rad_2_deg;
  }

  tf::Quaternion SnapToRightAngle(const tf::Quaternion& rotation)
  {
    if (rotation == tf::Quaternion::getIdentity())
    {
      return rotation;
    }

    tf::Matrix3x3 matrix(rotation);

    // First determine the order to process the rows in.  Rows with the largest
    // absolute max values will be ordered first.
    std::vector<std::pair<int32_t, double> > process_order(3);
    for (int32_t i = 0; i < 3; i++)
    {
      process_order[i].first = i;

      tf::Vector3 row = matrix.getRow(i).absolute();
      process_order[i].second = row[row.maxAxis()];
    }
    std::sort(process_order.begin(), process_order.end(), compare_rows);

    // Update the rotation matrix by operating on each row in the determined
    // order.  Each row will be aligned to its primary axis such that a single
    // element in each row will be either 1 or -1 and the rest will be 0.
    for (int32_t i = 0; i < 3; i++)
    {
      int32_t row_num = process_order[i].first;
      tf::Vector3 row = GetPrimaryAxis(matrix.getRow(row_num));

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
       return tf::Quaternion::getIdentity();
    }

    tf::Quaternion snapped_rotation;
    matrix.getRotation(snapped_rotation);

    return snapped_rotation;
  }

  tf::Vector3 GetPrimaryAxis(const tf::Vector3& vector)
  {
    tf::Vector3 vector_out = vector;

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
          vector_out[i] = 1.0 * boost::math::sign<double>(vector[i]);
        }
        else
        {
          vector_out[i] = 0;
        }
      }
    }

    return vector_out;
  }

  bool IsRotation(tf::Matrix3x3 matrix)
  {
    // Check that determinant is near 1.
    if (!math_util::IsNear(matrix.determinant(), 1, 0.00001))
    {
      return false;
    }

    // Check that the each row is a unit vector.
    for (int32_t i = 0; i < 3; i++)
    {
      if (!math_util::IsNear(matrix.getRow(i).length(), 1, 0.00001))
      {
        return false;
      }
    }

    // Check that the each column is a unit vector.
    for (int32_t i = 0; i < 3; i++)
    {
      if (!math_util::IsNear(matrix.getColumn(i).length(), 1, 0.00001))
      {
        return false;
      }
    }

    return true;
  }

  tf::Matrix3x3 GetUpperLeft (const boost::array<double, 36>& matrix)
  {
    tf::Matrix3x3 sub_matrix;

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

  tf::Matrix3x3 GetLowerRight (const boost::array<double, 36>& matrix)
  {
    tf::Matrix3x3 sub_matrix;

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

  void SetUpperLeft (
      const tf::Matrix3x3& sub_matrix,
      boost::array<double, 36>& matrix)
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

  void SetLowerRight (
      const tf::Matrix3x3& sub_matrix,
      boost::array<double, 36>& matrix)
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
}
