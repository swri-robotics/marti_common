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

#ifndef MATH_UTIL_MATH_UTIL_H_
#define MATH_UTIL_MATH_UTIL_H_

#include <math_util/constants.h>
namespace math_util
{
  /**
   * Round the value to the nearest integer.
   *
   * @param[in]  value     The number to round.
   * @param[in]  multiple  The multiple.
   *
   * @returns The rounded value.
   */
  double Round(double value);

  /**
   * Round the value to the nearest provided multiple.
   *
   * @param[in]  value     The number to round.
   * @param[in]  multiple  The multiple.
   *
   * @returns The rounded value.
   */
  double ToNearest(double value, double multiple);

  /**
   * Check if v1 is within +/- epsilon of v2
   *
   * @param[in]  v1       The first value.
   * @param[in]  v2       The second value.
   * @param[in]  epsilon  The tolerance.
   *
   * @returns True if v1 is near v2.
   */
  bool IsNear(double v1, double v2, double epsilon);

  /**
   * Unwraps the variable_angle across 0-2pi or +/-pi boundaries to avoid large
   * differences between the static and variable angles (when the difference may
   * be small.  For example a static angle of 2pi - epsilon and a variable angle
   * of epsilon would result in the variable angle being unwrapped to 2pi +
   * epsilon.
   *
   * @param[in]  static_angle     The reference angle
   * @param[in]  variable_angle   The angle to unwrap
   * @param[in]  threshold        A threshold on the angle (default is pi)
   *
   * @retval     Returns the resultant angle
   */
  double unWrapAngle(double static_angle,
                     double variable_angle,
                     double threshold = _pi);

  /**
   * Forces the angle to be between 0 and 2pi
   *
   * @param[in]  radians    The angle to fix in radians
   *
   * @retval     The fixed angle
   */
  double fixAngle0to2Pi(double radians);


  /**
   * Forces the angle to be between 0 and 2pi
   *
   * @param[in]  radians    The angle to fix in radians
   *
   * @retval     The fixed angle
   */
  double fixAngleMinusPitoPi(double radians)
  {
    // TODO(kkozak): Check this implementation
    return fixAngle0to2Pi(radians + _pi) - _pi;
  }

}

#endif  // MATH_UTIL_TRIG_UTIL_H_
