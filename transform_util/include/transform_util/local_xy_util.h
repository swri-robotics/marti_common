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

#ifndef TRANSFORM_UTIL_LOCAL_XY_UTIL_H_
#define TRANSFORM_UTIL_LOCAL_XY_UTIL_H_

#include <string>

#include <boost/shared_ptr.hpp>

namespace transform_util
{
/**
 * Transform a point from WGS84 lat/lon to an ortho-rectified LocalXY coordinate
 * system.
 *
 * @param[in] latitude             The input latitude in degrees.
 * @param[in] longitude            The input latitude in degrees.
 * @param[in] reference_latitude   The reference WGS84 latitude in degrees.
 * @param[in] reference_longitude  The reference WGS84 longitude in degrees.
 * @param[out] x                   The output X coordinate in meters.
 * @param[out] y                   The output Y coordinate in meters.
 */
  void LocalXyFromWgs84(
      double latitude,
      double longitude,
      double reference_latitude,
      double reference_longitude,
      double& x,
      double& y);

  /**
   * Transform a point from an ortho-rectified LocalXY coordinate system into
   * WGS84 latitude and longitude.
   *
   * Assumes the LocalXY data was generated with respect to the WGS84 datum.
   *
   * @param[in]  x                    The input X coordinate in meters.
   * @param[in]  y                    The input Y coordinate in meters.
   * @param[in]  reference_latitude   The reference WGS84 latitude in degrees.
   * @param[in]  reference_longitude  The reference WGS84 longitude in degrees.
   * @param[out] latitude             The output latitude in degrees.
   * @param[out] longitude            The output latitude in degrees.
   */
  void Wgs84FromLocalXy(
      double x,
      double y,
      double reference_latitude,
      double reference_longitude,
      double& latitude,
      double& longitude);

  /**
   * Utility class for converting between WGS84 lat/lon and an ortho-rectified
   * LocalXY coordinate system.
   */
  class LocalXyWgs84Util
  {
  public:
    /**
     * Constructor.
     *
     * @param[in] reference_latitude  Reference latitude in degrees.
     * @param[in] reference_longitude  Reference longitude in degrees.
     * @param[in] reference_altitude  Reference altitude in meters.
     * @param[in] reference_heading  Reference heading in degrees.
     */
    LocalXyWgs84Util(
        double reference_latitude,
        double reference_longitude,
        double reference_heading = 0,
        double reference_altitude = 0,
        const std::string& frame_id = std::string(""));
    // TODO(malban): What is the heading referenced from?

    double ReferenceLongitude() const;

    double ReferenceLatitude() const;

    double ReferenceHeading() const;

    double ReferenceAltitude() const;

    std::string FrameId() const;

    /**
     * Convert WGS84 latitude and longitude to LocalXY.
     *
     * @param[in]  latitude   Latitude value in degrees.
     * @param[in]  longitude  Longitude value in degrees.
     * @param[out] x          X coordinate in meters from origin.
     * @param[out] y          Y coordinate in meters from origin.
     *
     * @returns True if the conversion is possible.
     */
    bool ToLocalXy(
        double latitude,
        double longitude,
        double& x,
        double& y) const;

    /**
     * Convert LocalXY to WGS84 latitude and longitude.
     *
     * @param[in]  x          X coordinate in meters from origin.
     * @param[in]  y          Y coordinate in meters from origin.
     * @param[out] latitude   Latitude value in degrees.
     * @param[out] longitude  Longitude value in degrees.
     */
    void ToWgs84(
        double x,
        double y,
        double& latitude,
        double& longitude) const;

  protected:
    double reference_latitude_;   //< Reference latitude in radians.
    double reference_longitude_;  //< Reference longitude in radians.
    double reference_heading_;    //< Reference heading in radians.
    double reference_altitude_;   //< Reference altitude in meters.

    std::string frame_id_;

    double rho_lat_;
    double rho_lon_;
    double cos_heading_;
    double sin_heading_;
  };
  typedef boost::shared_ptr<LocalXyWgs84Util> LocalXyWgs84UtilPtr;

  LocalXyWgs84UtilPtr ParseLocalXyOrigin(bool* waiting_for_auto_origin = 0);
}

#endif  // TRANSFORM_UTIL_LOCAL_XY_UTIL_H_
