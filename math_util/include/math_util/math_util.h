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

#ifndef MATH_UTIL_MATH_UTIL_H_
#define MATH_UTIL_MATH_UTIL_H_

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
  double ToNearest(double value, int multiple);
}

#endif  // MATH_UTIL_TRIG_UTIL_H_
