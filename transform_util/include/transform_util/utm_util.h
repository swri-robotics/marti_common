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

#include <boost/serialization/singleton.hpp>
#include <boost/thread/mutex.hpp>

#include <proj_api.h>

namespace transform_util
{
  uint32_t GetZone(double longitude);

  char GetBand(double latitude);

  class UtmUtil
  {
  public:
    UtmUtil();

    /**
     * Convert WGS84 latitude and longitude to UTM.
     * 
     * @param[in]  latitude   Latitude value in degrees.
     * @param[in]  longitude  Longitude value in degrees.
     * @param[out] zone       UTM zone.
     * @param[out] band       UTM band.
     * @param[out] easting    UTM easting in meters.
     * @param[out] northing   UTM northing in meters.
     */
    void ToUtm(
      double latitude, double longitude,
      int& zone, char& band, double& easting, double& northing) const;

    /**
     * Convert WGS84 latitude and longitude to UTM.
     * 
     * @param[in]  latitude   Latitude value in degrees.
     * @param[in]  longitude  Longitude value in degrees.
     * @param[out] easting    UTM easting in meters.
     * @param[out] northing   UTM northing in meters.
     */
    void ToUtm(
      double latitude, double longitude,
      double& easting, double& northing) const;

    /**
     * Convert UTM easting and northing to WGS84 latitude and longitude.
     *
     * @param[in]  zone       UTM zone.
     * @param[in]  band       UTM band.
     * @param[in]  easting    UTM easting in meters.
     * @param[in]  northing   UTM northing in meters.
     * @param[out] latitude   WGS84 latitude in degrees.
     * @param[out] longitude  WGS84 longitude in degrees.
     */
    void ToLatLon(
      int zone, char band, double easting, double northing,
      double& latitude, double& longitude) const;

  private:
    /**
     * The actual UTM conversion processing takes place in this helper class
     * which is a singlton due to the large memory footprint of the underlying
     * PROJ.4 projections library structures.  Thread safety is enforced with
     * mutexes around the PROJ.4 functions, but could be achieved in the future
     * with a thread-safe version of PROJ.4 or ignored all together if the calls
     * can be determined to be thread safe in this context.
     */
    class UtmData
    {
      public:
        ~UtmData();

        void ToUtm(
          double latitude, double longitude,
          int& zone, char& band, double& easting, double& northing) const;

        void ToUtm(
          double latitude, double longitude,
          double& easting, double& northing) const;

        void ToLatLon(
          int zone, char band, double easting, double northing,
          double& latitude, double& longitude) const;

        friend class boost::serialization::detail::singleton_wrapper<transform_util::UtmUtil::UtmData>;
      private:
        UtmData();

        projPJ lat_lon_;
        projPJ utm_north_[60];
        projPJ utm_south_[60];

        mutable boost::mutex mutex_;
    };
    typedef boost::serialization::singleton<UtmData> UtmDataSingleton;

    const UtmData& utm_data_;
  };
}

#endif  // TRANSFORM_UTIL_UTM_UTIL_H_
