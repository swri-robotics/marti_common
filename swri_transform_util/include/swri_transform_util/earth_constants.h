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

#ifndef TRANSFORM_UTIL_EARTH_CONSTANTS_H_
#define TRANSFORM_UTIL_EARTH_CONSTANTS_H_

namespace swri_transform_util
{
  /**
   * Earth equatorial radius in meters according to WGS84.
   */
  static const double _earth_equator_radius = 6378137.0;

  /**
   * Earth mean radius in meters
   */
  static const double _earth_mean_radius = 6371009.0;

  /**
   * Earth equatorial circumference in meters according to WGS84.
   */
  static const double _earth_equator_circumference = 40075016.69;

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

  /**
   * Earth rotation rate in radians per second.
   */
  static const double _earth_rotation_rate = 7.292115e-5;
}

#endif  // TRANSFORM_UTIL_EARTH_CONSTANTS_H_
