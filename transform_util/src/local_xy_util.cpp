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
    reference_altitude_(reference_altitude),
    rho_lat_(0),
    rho_lon_(0),
    cos_heading_(0),
    sin_heading_(0),
    frame_(""),
    initialized_(false)
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
    frame_(""),
    initialized_(false)
  {
    ros::NodeHandle node;

    if (node.getParam("/local_xy_frame", frame_)) {
      ROS_INFO("Using XY frame at /local_xy_frame: %s", frame_.c_str());
    }
    
    ROS_INFO("Subscribing to /local_xy_origin");
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
    if (!initialized_)
    {
      reference_latitude_ = origin->latitude * math_util::_deg_2_rad;
      reference_longitude_ = origin->longitude * math_util::_deg_2_rad;
      reference_altitude_ = origin->altitude;

      if (frame_.empty()) {
        frame_ = origin->header.frame_id;
      } else if (frame_ != origin->header.frame_id) {
        ROS_WARN("local_xy_frame (%s) was set by parameter and "
                 "does not match local_xy_origin header (%s).",
                 frame_.c_str(),
                 origin->header.frame_id.c_str());        
      }
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
