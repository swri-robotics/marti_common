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

#ifndef TRANSFORM_UTIL_LOCAL_XY_UTIL_H_
#define TRANSFORM_UTIL_LOCAL_XY_UTIL_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <swri_transform_util/transform_util.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "GeographicLib/LocalCartesian.hpp"

namespace swri_transform_util
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
   *
   * To use this class, first construct it with a reference origin. The
   * reference origin should be a latitude, longitude, angle, and altitude in
   * WGS84 coordinates. Once initialized, a LocalXyWgs84Util can be used to
   * convert WGS84 coordinates to and from an ortho-rectified frame with its
   * origin at the reference origin. Because the earth is spherical, the error
   * in the ortho-rectified frame will accumulate as the distance from the
   * reference origin increases. For this reason, the reference origin should
   * be chosen to be close to the region of interest (<10 km).
   *
   * It is strongly recommended to use 0 degrees for the angle. This
   * corresponds to the X-axis of the ortho-rectified frame pointing east.
   *
   */
  class LocalXyWgs84Util
  {
  public:
    /**
     * Initializing constructor
     *
     * This constructor creates and initializes a LocalXyWgs84Util.
     *
     * @param[in] reference_latitude   Reference latitude in degrees.
     * @param[in] reference_longitude  Reference longitude in degrees.
     * @param[in] reference_angle      Reference angle in degrees ENU.
     * @param[in] reference_altitude   Reference altitude in meters.
     */
    LocalXyWgs84Util(
        double reference_latitude,
        double reference_longitude,
        double reference_angle = 0,
        double reference_altitude = 0,
        rclcpp::Node::SharedPtr node = nullptr);

    /**
     * Zero-argument constructor.
     *
     * This constructor creates an uninitialized LocalXyWgs84Util. This
     * constructor is only used to create placeholder objects in containers
     * that require a zero-argument constructor.
     */
    explicit LocalXyWgs84Util(rclcpp::Node::SharedPtr node);

    /**
     * Return whether the object has been initialized
     *
     * The object is not usable unless it has been initialized (see the two
     * constructors).
     *
     * @return True if initialized, false otherwise.
     */
    bool Initialized() const { return initialized_; }

    /**
     * Reset to "not Initialized". Useful when the local_xy_origin
     * changes and we want this class to be updated.
     */
    void ResetInitialization();

    /**
     * Return the longitude coordinate of the local origin
     *
     * @return The WGS84 longitude coordinate of the local origin in degrees
     */
    double ReferenceLongitude() const;

    /**
     * Return the latitude coordinate of the local origin
     *
     * @return The WGS84 latitude coordinate of the local origin in degrees
     */
    double ReferenceLatitude() const;

    /**
     * Return the reference angle in degrees ENU.
     */
    double ReferenceAngle() const;

    /**
     * Return the altitude coordinate of the local origin
     *
     * @return The WGS84 altitude coordinate of the local origin in meters
     */
    double ReferenceAltitude() const;

    /**
     * Return the TF frame ID corresponding to the local origin
     *
     * @return The TF frame ID corresponding to the local origin
     */
    std::string Frame() const { return frame_; }

    /**
     * Return the TF frame ID corresponding to the local origin with a leading slash
     *
     * @return The TF frame ID corresponding to the local origin with a leading slash
     */
    std::string NormalizedFrame() const { return NormalizeFrameId(frame_); }

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
     *
     * @returns True if the conversion is possible.
     */
    bool ToWgs84(
        double x,
        double y,
        double& latitude,
        double& longitude) const;

  protected:
    rclcpp::Node::SharedPtr node_;

    double reference_angle_;      //< Reference angle in radians ENU.

    GeographicLib::LocalCartesian local_cartesian_;

    double cos_angle_;
    double sin_angle_;

    std::string frame_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    bool initialized_;

    void Initialize();

    void HandleOrigin(double latitude, double longitude, double altitude, double angle, const std::string& frame_id);

    void HandlePoseStamped(geometry_msgs::msg::PoseStamped::UniquePtr pose);
  };
  typedef std::shared_ptr<LocalXyWgs84Util> LocalXyWgs84UtilPtr;
}

#endif  // TRANSFORM_UTIL_LOCAL_XY_UTIL_H_
