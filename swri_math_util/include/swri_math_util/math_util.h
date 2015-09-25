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

#ifndef MATH_UTIL_MATH_UTIL_H_
#define MATH_UTIL_MATH_UTIL_H_

#include <swri_math_util/constants.h>
namespace swri_math_util
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
   * Round up the value to the nearest provided multiple.
   *
   * @param[in]  value     The number to round.
   * @param[in]  multiple  The multiple.
   *
   * @returns The rounded value.
   */
  double UpToNearest(double value, double multiple);

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
}

#endif  // MATH_UTIL_MATH_UTIL_H_
