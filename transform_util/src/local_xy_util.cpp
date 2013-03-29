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

#include <transform_util/local_xy_util.h>

#include <cmath>

#include <boost/make_shared.hpp>

#include <XmlRpcException.h>

#include <ros/ros.h>

#include <math_util/constants.h>
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
      double reference_altitude,
      const std::string& frame_id) :
    reference_latitude_(reference_latitude * math_util::_deg_2_rad),
    reference_longitude_(reference_longitude * math_util::_deg_2_rad),
    reference_heading_(reference_heading * math_util::_deg_2_rad),
    reference_altitude_(reference_altitude),
    frame_id_(frame_id)
  {
    if (reference_heading_ > math_util::_pi)
    {
      reference_heading_ -= math_util::_2pi;
    }
    else if (reference_heading_ <= -math_util::_pi)
    {
      reference_heading_ += math_util::_2pi;
    }

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

  std::string LocalXyWgs84Util::FrameId() const
  {
    return frame_id_;
  }

  bool LocalXyWgs84Util::ToLocalXy(
      double latitude,
      double longitude,
      double& x,
      double& y) const
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

  void LocalXyWgs84Util::ToWgs84(
      double x,
      double y,
      double& latitude,
      double& longitude) const
  {
    double dLon = cos_heading_ * x + sin_heading_ * y;
    double dLat = (y - dLon * sin_heading_) / cos_heading_;
    double rlat = (dLat / rho_lat_) + reference_latitude_;
    double rlon = (dLon / rho_lon_) + reference_longitude_;

    latitude = rlat * math_util::_rad_2_deg;
    longitude = rlon * math_util::_rad_2_deg;
  }

  LocalXyWgs84UtilPtr ParseLocalXyOrigin()
  {
    LocalXyWgs84UtilPtr local_xy;

    try
    {
      std::string local_xy_frame;
      if (!ros::param::get("/local_xy_frame", local_xy_frame))
      {
        ROS_ERROR("[local_xy]: Undefined /local_xy_frame parameter.");
        return local_xy;
      }

      std::string local_xy_name;
      if (!ros::param::get("/local_xy_origin", local_xy_name))
      {
        ROS_ERROR("[local_xy]: Undefined /local_xy_origin parameter.");
        return local_xy;
      }

      XmlRpc::XmlRpcValue local_xy_origins;
      if (!ros::param::get("/local_xy_origins", local_xy_origins))
      {
        ROS_ERROR("[local_xy]: Undefined /local_xy_origins parameter.");
        return local_xy;
      }

      if (local_xy_origins.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[local_xy]: Invalid /local_xy_origins parameter.");
        return local_xy;
      }

      for (int32_t i = 0; i < local_xy_origins.size(); ++i)
      {
        if (local_xy_origins[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR("[local_xy]: Invalid /local_xy_origins parameter.");
          return local_xy;
        }

        if (!local_xy_origins[i].hasMember("name"))
        {
          ROS_ERROR("[local_xy]: Invalid /local_xy_origins parameter: Missing name");
          return local_xy;
        }

        std::string name = static_cast<std::string>(local_xy_origins[i]["name"]);

        if (name == local_xy_name)
        {
          if (!local_xy_origins[i].hasMember("latitude") ||
              !local_xy_origins[i].hasMember("longitude") ||
              !local_xy_origins[i].hasMember("altitude") ||
              !local_xy_origins[i].hasMember("heading"))
          {
            ROS_ERROR("[local_xy]: Invalid /local_xy_origins parameter.");
            return local_xy;
          }

          double lat = static_cast<double>(local_xy_origins[i]["latitude"]);
          if (lat < -90 || lat > 90)
          {
            ROS_ERROR("[local_xy]: Invalid LocalXY latitude: Out of range [-90, 90]");
            return local_xy;
          }

          double lon = static_cast<double>(local_xy_origins[i]["longitude"]);
          if (lon < -180 || lon > 180)
          {
            ROS_ERROR("[local_xy]: Invalid LocalXY longitude: Out of range [-180, 180]");
            return local_xy;
          }

          double alt = static_cast<double>(local_xy_origins[i]["altitude"]);
          double heading = static_cast<double>(local_xy_origins[i]["heading"]);

          local_xy = boost::make_shared<LocalXyWgs84Util>(
              lat, lon, heading, alt, local_xy_frame);
        }
      }
    }
    catch (const XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("[local_xy]: Exception parsing LocalXY origin parameters: %s",
          e.getMessage().c_str());
    }
    catch(...)
    {
      ROS_ERROR("[local_xy]: Exception parsing LocalXY origin parameters.");
      return local_xy;
    }

    return local_xy;
  }
}
