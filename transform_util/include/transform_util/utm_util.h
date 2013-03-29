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

#ifndef TRANSFORM_UTIL_UTM_UTIL_H_
#define TRANSFORM_UTIL_UTM_UTIL_H_

#include <stdint.h>
#include <proj_api.h>

namespace transform_util
{
  uint32_t GetZone(double longitude);

  char GetBand(double latitude);

  class UtmUtil
  {
  public:
    UtmUtil();
    ~UtmUtil();

    /**
     * @brief Convert WGS84 latitude and longitude to UTM.
     * @param[in]  latitude   Latitude value.
     * @param[in]  longitude  Longitude value.
     * @param[out] zone       UTM zone.
     * @param[out] band       UTM band.
     * @param[out] easting    UTM easting.
     * @param[out] northing   UTM northing.
     */
    void ToUtm(double latitude, double longitude, int& zone, char& band, double& easting, double& northing) const;

    /**
     * @brief Convert WGS84 latitude and longitude to UTM.
     * @param[in]  latitude   Latitude value.
     * @param[in]  longitude  Longitude value.
     * @param[out] easting    UTM easting.
     * @param[out] northing   UTM northing.
     */
    void ToUtm(double latitude, double longitude, double& easting, double& northing) const;

    /**
     * @brief Convert UTM easting and northing to WGS84 latitude and longitude.
     * @param[in]  zone       UTM zone.
     * @param[in]  band       UTM band.
     * @param[in]  easting    UTM easting.
     * @param[in]  northing   UTM northing.
     * @param[out] latitude   WGS84 latitude.
     * @param[out] longitude  WGS84 longitude.
     */
    void ToLatLon(int zone, char band, double easting, double northing, double& latitude, double& longitude) const;

    projPJ lat_lon_;
    projPJ utm_north_[60];
    projPJ utm_south_[60];

  private:
    explicit UtmUtil(UtmUtil const&);
    void operator=(UtmUtil const&);
  };
}

#endif  // TRANSFORM_UTIL_UTM_UTIL_H_
