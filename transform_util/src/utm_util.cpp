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

#include <transform_util/utm_util.h>

#include <cmath>

#include <ros/ros.h>

#include <math_util/constants.h>

namespace transform_util
{
  uint32_t GetZone(double longitude)
  {
    int32_t zone = static_cast<int32_t>((longitude + 180.0) / 6.0) + 1;
    if (zone < 1) zone = 1;
    if (zone > 60) zone = 60;

    return static_cast<uint32_t>(zone);
  }

  char GetBand(double latitude)
  {
    // This routine determines the correct UTM letter designator for the given
    // latitude returns 'Z' if latitude is outside the UTM limits of 84N to 80S
    // Written by Chuck Gantz- chuck.gantz@globalstar.com

    char band;

    if (latitude > 84) band = 'Z';
    else if (latitude >= 72)  band = 'X';
    else if (latitude >= 64)  band = 'W';
    else if (latitude >= 56)  band = 'V';
    else if (latitude >= 48)  band = 'U';
    else if (latitude >= 40)  band = 'T';
    else if (latitude >= 32)  band = 'S';
    else if (latitude >= 24)  band = 'R';
    else if (latitude >= 16)  band = 'Q';
    else if (latitude >= 8)   band = 'P';
    else if (latitude >= 0)   band = 'N';
    else if (latitude >= -8)  band = 'M';
    else if (latitude >= -16) band = 'L';
    else if (latitude >= -24) band = 'K';
    else if (latitude >= -32) band = 'J';
    else if (latitude >= -40) band = 'H';
    else if (latitude >= -48) band = 'G';
    else if (latitude >= -56) band = 'F';
    else if (latitude >= -64) band = 'E';
    else if (latitude >= -72) band = 'D';
    else if (latitude >= -80) band = 'C';
    else
      band = 'Z';

    return band;
  }

  UtmUtil::UtmUtil()
  {
    // Initialize lat long projection.
    lat_lon_ = pj_init_plus("+proj=latlong +ellps=WGS84");

    // Initialize projection for each UTM zone.
    char args[64];
    for (int i = 0; i < 60; i++)
    {
      sprintf(args, "+proj=utm +ellps=WGS84 +zone=%d", i + 1);
      utm_north_[i] = pj_init_plus(args);

      sprintf(args, "+proj=utm +ellps=WGS84 +zone=%d +south", i + 1);
      utm_south_[i] = pj_init_plus(args);
    }
  }

  UtmUtil::~UtmUtil()
  {
    pj_free(lat_lon_);

    // Cleanup projection memory.
    for (int i = 0; i < 60; i++)
    {
      pj_free(utm_north_[i]);
      pj_free(utm_south_[i]);
    }
  }

  void UtmUtil::ToUtm(
      double latitude,
      double longitude,
      int& zone,
      char& band,
      double& easting,
      double& northing) const
  {
    zone = GetZone(longitude);
    band = GetBand(latitude);

    double x = longitude * math_util::_deg_2_rad;
    double y = latitude * math_util::_deg_2_rad;

    // Get easting and northing values.
    if (band <= 'N')
    {
      pj_transform(lat_lon_, utm_south_[zone - 1], 1, 0, &x, &y, NULL);
    }
    else
    {
      pj_transform(lat_lon_, utm_north_[zone - 1], 1, 0, &x, &y, NULL);
    }

    easting = x;
    northing = y;
  }

  void UtmUtil::ToUtm(
      double latitude,
      double longitude,
      double& easting,
      double& northing) const
  {
    int zone;
    char band;

    ToUtm(latitude, longitude, zone, band, easting, northing);
  }

  void UtmUtil::ToLatLon(
      int zone,
      char band,
      double easting,
      double northing,
      double& latitude,
      double& longitude) const
  {
    double x = easting;
    double y = northing;

    if (band <= 'N')
    {
      pj_transform(utm_south_[zone - 1], lat_lon_, 1, 0, &x, &y, NULL);
    }
    else
    {
      pj_transform(utm_north_[zone - 1], lat_lon_, 1, 0, &x, &y, NULL);
    }

    longitude = x * math_util::_rad_2_deg;
    latitude = y * math_util::_rad_2_deg;
  }
}
