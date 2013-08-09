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

#include <transform_util/local_xy_util.h>

#include <cmath>

#include <boost/make_shared.hpp>

#include <math_util/constants.h>
#include <math_util/trig_util.h>
#include <transform_util/earth_constants.h>

namespace transform_util
{
  void LocalXyFromWgs84(
      double latitude,
      double longitude,
      double reference_latitude,
      double reference_longitude,
      double& x,
      double& y)
  {
    LocalXyWgs84Util local_xy(reference_latitude, reference_longitude);
    local_xy.ToLocalXy(latitude, longitude, x, y);
  }

  void Wgs84FromLocalXy(
      double x,
      double y,
      double reference_latitude,
      double reference_longitude,
      double& latitude,
      double& longitude)
  {
    LocalXyWgs84Util local_xy(reference_latitude, reference_longitude);
    local_xy.ToWgs84(x, y, latitude, longitude);
  }

  LocalXyWgs84Util::LocalXyWgs84Util(
      double reference_latitude,
      double reference_longitude,
      double reference_heading,
      double reference_altitude) :
    reference_latitude_(reference_latitude * math_util::_deg_2_rad),
    reference_longitude_(reference_longitude * math_util::_deg_2_rad),
    reference_heading_(reference_heading * math_util::_deg_2_rad),
    reference_altitude_(reference_altitude)
  {
    Initialize();
  }

  LocalXyWgs84Util::LocalXyWgs84Util() :
    reference_latitude_(0),
    reference_longitude_(0),
    reference_heading_(0),
    reference_altitude_(0),
    rho_lat_(0),
    rho_lon_(0),
    cos_heading_(0),
    sin_heading_(0),
    initialized_(false)
  {
    ros::NodeHandle node;

    ROS_ERROR("Subscribing to /local_xy_origin");
    origin_sub_ = node.subscribe("/local_xy_origin", 1, &LocalXyWgs84Util::HandleOrigin, this);
  }

  void LocalXyWgs84Util::Initialize()
  {
    reference_heading_ = math_util::WrapRadians(reference_heading_, 0);

    cos_heading_ = std::cos(reference_heading_);
    sin_heading_ = std::sin(reference_heading_);

    double depth = -reference_altitude_;

    double p = _earth_eccentricity * std::sin(reference_latitude_);
    p = 1.0 - p * p;

    double rho_e = _earth_equator_radius *
        (1.0 - _earth_eccentricity * _earth_eccentricity) / (std::sqrt(p) * p);
    double rho_n = _earth_equator_radius / std::sqrt(p);

    rho_lat_ = rho_e - depth;
    rho_lon_ = (rho_n - depth) * std::cos(reference_latitude_);
    initialized_ = true;
  }

  void LocalXyWgs84Util::HandleOrigin(const gps_common::GPSFixConstPtr origin)
  {
    ROS_ERROR("HandleOrigin");
    if (!initialized_)
    {
      reference_latitude_ = origin->latitude * math_util::_deg_2_rad;
      reference_longitude_ = origin->longitude * math_util::_deg_2_rad;
      reference_altitude_ = origin->altitude;
      Initialize();
    }
    origin_sub_.shutdown();
  }

  double LocalXyWgs84Util::ReferenceLongitude() const
  {
    return reference_longitude_ * math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceLatitude() const
  {
    return reference_latitude_ * math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceHeading() const
  {
    return reference_heading_ * math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceAltitude() const
  {
    return reference_altitude_;
  }

  bool LocalXyWgs84Util::ToLocalXy(
      double latitude,
      double longitude,
      double& x,
      double& y) const
  {
    if (initialized_)
    {
      if (latitude < -90.0 || latitude > 90.0)
      {
        return false;
      }

      if (longitude < -180.0 || longitude > 180.0)
      {
        return false;
      }

      double rlat = latitude * math_util::_deg_2_rad;
      double rlon = longitude * math_util::_deg_2_rad;
      double dLat = (rlat - reference_latitude_) * rho_lat_;
      double dLon = (rlon - reference_longitude_) * rho_lon_;

      y = dLat * cos_heading_ + dLon * sin_heading_;
      x = -1.0 * (dLat * sin_heading_ - dLon * cos_heading_);

      return true;
    }

    return initialized_;
  }

  bool LocalXyWgs84Util::ToWgs84(
      double x,
      double y,
      double& latitude,
      double& longitude) const
  {
    if (initialized_)
    {
      double dLon = cos_heading_ * x + sin_heading_ * y;
      double dLat = (y - dLon * sin_heading_) / cos_heading_;
      double rlat = (dLat / rho_lat_) + reference_latitude_;
      double rlon = (dLon / rho_lon_) + reference_longitude_;

      latitude = rlat * math_util::_rad_2_deg;
      longitude = rlon * math_util::_rad_2_deg;
    }

    return initialized_;
  }
}
