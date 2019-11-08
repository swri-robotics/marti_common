// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
#ifndef MATH_UTIL_INTERPOLATION_1D_H_
#define MATH_UTIL_INTERPOLATION_1D_H_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace swri_math_util
{
  class Interpolation1D
  {
  public:
    enum InterpolationType
    {
      ZERO_ORDER_HOLD,
      LINEAR
    };

    Interpolation1D(rclcpp::Node& node);

    bool appendPoint(double x, double y);

    size_t numPoints() const;

    std::pair<double, double> getPoint(size_t index) const;

    void removePoint(size_t index);

    void clear();

    InterpolationType interpolationType();

    std::string interpolationTypeString() const;

    void setInterpolationType(InterpolationType type);

    bool readFromParameter(const std::string& param_name);

    double minX() const;

    double maxX() const;

    double eval(double x) const;

  private:
    rclcpp::Node& node_;
    InterpolationType interp_type_;
    std::vector<double> x_;
    std::vector<double> y_;
  };

  inline
  double Interpolation1D::eval(double x) const
  {
    if (x_.size() == 0)
    {
      // If we have no points, we just return a sensible default value.
      return 0.0;
    }
    else if (x_.size() == 1)
    {
      // If we have a single point, we can't do any interpolation so we
      // just return the single output value.
      return y_[0];
    }
    else if (x <= x_.front())
    {
      // Clamp the output to the first output if we are below the
      // domain.
      return y_.front();
    }
    else if (x >= x_.back())
    {
      // Clamp the output to the last output if we are above the domain.
      return y_.back();
    }

    // If we pass the general special cases, we have at least two points
    // that bound the evaluation point.  First, we find the index of the
    // point below the evaluation point, and then perform the
    // interpolation.

    // We are searching for the interval where x_i <= x <= x_{i+1}.  The
    // first interval has index 0, the last has index x_.size()-2.
    size_t i_min = 0;
    size_t i_max = x_.size() - 2;
    size_t i_mid = 0;

    while (i_min <= i_max)
    {
      i_mid = i_min + (i_max - i_min) / 2;

      if (x_[i_mid] <= x && x_[i_mid + 1] >= x)
      {
        // This is the interval that contains our point, so stop searching.
        break;
      }
      else if (x < x_[i_mid])
      {
        // The desired interval must be below this one.
        i_max = i_mid - 1;
      }
      else
      {
        i_min = i_mid + 1;
      }
    }
    // The desired interval is at i_mid.

    if (interp_type_ == ZERO_ORDER_HOLD)
    {
      return y_[i_mid];
    }
    else if (interp_type_ == LINEAR)
    {
      size_t i0 = i_mid;
      size_t i1 = i_mid + 1;
      double s = (x - x_[i0]) / (x_[i1] - x_[i0]);
      return (1.0 - s) * y_[i0] + s * y_[i1];
    }
    else
    {
      // We should always have a valid interpolation type, but just in
      // case we print out an error and use a zero order hold.
      RCLCPP_ERROR(node_.get_logger(), "Invalid interpolation type: %d", interp_type_);
      return y_[i_mid];
    }
  }
}  // namespace swri_math_util
#endif  // MATH_UTIL_INTERPOLATION_1D_H_


