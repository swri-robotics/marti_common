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

#ifndef TRANSFORM_UTIL_EARTH_CONSTANTS_H_
#define TRANSFORM_UTIL_EARTH_CONSTANTS_H_

namespace transform_util
{
  /**
   * Earth equatorial radius in meters according to WGS84.
   */
  static const double _earth_equator_radius = 6378137.0;

  /**
   * Earth 'first' eccentricity according to WGS84.
   */
  static const double _earth_eccentricity = 0.08181919084261;

  /**
   * Earth flattening according to WGS84.
   *
   * Flattening is a measure of the compression of a sphere along a diameter to
   * form an ellipsoid of revolution.
   *
   * See: http://en.wikipedia.org/wiki/Flattening
   */
  static const double _earth_flattening = 3.35281066475e-3;

  static const double _earth_rotation_rate = 7.292115e-5;     //< rad/sec

}

#endif  // TRANSFORM_UTIL_EARTH_CONSTANTS_H_
