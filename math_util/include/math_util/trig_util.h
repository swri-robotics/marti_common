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

#ifndef MATH_UTIL_TRIG_UTIL_H_
#define MATH_UTIL_TRIG_UTIL_H_

namespace math_util
{
  /**
   * Normalize an angle to be within a 2pi range centered at a given value.
   *
   * @param[in]:  angle   The input angle in radians.
   * @param[in]:  center  The center of the range in radians.
   *
   * @returns An equivalent angle in the desired range.
   */
  double WrapRadians(double angle, double center);
}

#endif  // MATH_UTIL_TRIG_UTIL_H_
