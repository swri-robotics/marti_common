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

#include <math_util/constants.h>

namespace transform_util
{
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
}
