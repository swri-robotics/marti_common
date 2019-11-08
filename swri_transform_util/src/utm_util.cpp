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

#include <swri_transform_util/utm_util.h>

#include <cmath>

#include <swri_math_util/constants.h>

namespace swri_transform_util
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

  UtmUtil::UtmData::UtmData()
  {
    // Initialize lat long projection.
    lat_lon_ = pj_init_plus("+proj=latlong +ellps=WGS84");

    // Initialize projection for each UTM zone.
    char args[64];
    for (int i = 0; i < 60; i++)
    {
      snprintf(args, sizeof(args), "+proj=utm +ellps=WGS84 +zone=%d", i + 1);
      utm_north_[i] = pj_init_plus(args);

      snprintf(args, sizeof(args), "+proj=utm +ellps=WGS84 +zone=%d +south", i + 1);
      utm_south_[i] = pj_init_plus(args);
    }
  }

  UtmUtil::UtmData::~UtmData()
  {
    pj_free(lat_lon_);

    // Cleanup projection memory.
    for (int i = 0; i < 60; i++)
    {
      pj_free(utm_north_[i]);
      pj_free(utm_south_[i]);
    }
  }

  void UtmUtil::UtmData::ToUtm(
      double latitude,
      double longitude,
      int& zone,
      char& band,
      double& easting,
      double& northing) const
  {
    boost::unique_lock<boost::mutex> lock(mutex_);

    zone = GetZone(longitude);
    band = GetBand(latitude);

    double x = longitude * swri_math_util::_deg_2_rad;
    double y = latitude * swri_math_util::_deg_2_rad;

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

  void UtmUtil::UtmData::ToUtm(
      double latitude,
      double longitude,
      double& easting,
      double& northing) const
  {
    int zone;
    char band;

    ToUtm(latitude, longitude, zone, band, easting, northing);
  }

  void UtmUtil::UtmData::ToLatLon(
      int zone,
      char band,
      double easting,
      double northing,
      double& latitude,
      double& longitude) const
  {
    boost::unique_lock<boost::mutex> lock(mutex_);

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

    longitude = x * swri_math_util::_rad_2_deg;
    latitude = y * swri_math_util::_rad_2_deg;
  }

  UtmUtil::UtmUtil() :
    utm_data_(UtmDataSingleton::get_const_instance())
  {
  }

  void UtmUtil::ToUtm(
      double latitude,
      double longitude,
      int& zone,
      char& band,
      double& easting,
      double& northing) const
  {
    utm_data_.ToUtm(latitude, longitude, zone, band, easting, northing);
  }

  void UtmUtil::ToUtm(
      double latitude,
      double longitude,
      double& easting,
      double& northing) const
  {
    utm_data_.ToUtm(latitude, longitude, easting, northing);
  }

  void UtmUtil::ToLatLon(
      int zone,
      char band,
      double easting,
      double northing,
      double& latitude,
      double& longitude) const
  {
    utm_data_.ToLatLon(zone, band, easting, northing, latitude, longitude);
  }
}
