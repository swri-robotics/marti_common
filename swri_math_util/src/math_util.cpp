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

#include <swri_math_util/math_util.h>

#include <cmath>

namespace swri_math_util
{
  double Round(double value)
  {
    return (value > 0.0) ? std::floor(value + 0.5) : std::ceil(value - 0.5);
  }

  double ToNearest(double value, double multiple)
  {
    if (multiple == 0)
    {
      return 0;
    }

    return Round(value / multiple) * multiple;
  }

  double UpToNearest(double value, double multiple)
  {
    if (multiple == 0)
    {
      return 0;
    }

    return std::ceil(value / multiple) * multiple;
  }

  bool IsNear(double v1, double v2, double epsilon)
  {
    return std::fabs(v1 - v2) <= epsilon;
  }

  double unWrapAngle(double static_angle,
                     double variable_angle,
                     double threshold)
  {
    if (std::abs(static_angle - variable_angle) > threshold)
    {
      if (variable_angle < static_angle)
      {
        variable_angle += swri_math_util::_2pi;
      }
      else
      {
        variable_angle -= swri_math_util::_2pi;
      }
    }
    return variable_angle;
  }
}
