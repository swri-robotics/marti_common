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
#include <swri_math_util/interpolation_1d.h>

namespace swri_math_util
{
  Interpolation1D::Interpolation1D(rclcpp::Node& node)
      :
      node_(node),
      interp_type_(ZERO_ORDER_HOLD)
  {
  }

  bool Interpolation1D::appendPoint(double x, double y)
  {
    if (x_.size() == 0 || x > x_.back())
    {
      // We can accept new points if it is the first point or greater
      // than the last point.
      x_.push_back(x);
      y_.push_back(y);
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_.get_logger(), "Error appending new point. "
                                       "X values must be increasing. (%f <= %f)",
                   x, x_.back());
      return false;
    }
  }

  size_t Interpolation1D::numPoints() const
  {
    return x_.size();
  }

  std::pair<double, double> Interpolation1D::getPoint(size_t index) const
  {
    if (index < x_.size())
    {
      return std::pair<double, double>(x_[index], y_[index]);
    }
    else
    {
      RCLCPP_ERROR(node_.get_logger(), "Invalid index in getPoint (index=%zu, numPoints=%zu)",
                   index, x_.size());
      return std::pair<double, double>(0.0, 0.0);
    }
  }

  void Interpolation1D::removePoint(size_t index)
  {
    if (index < x_.size())
    {
      x_.erase(x_.begin() + index);
      y_.erase(y_.begin() + index);
    }
    else
    {
      RCLCPP_ERROR(node_.get_logger(), "Invalid index in removePoint (index=%zu, numPoints=%zu)",
                   index, x_.size());
    }
  }

  void Interpolation1D::clear()
  {
    x_.clear();
    y_.clear();
  }

  Interpolation1D::InterpolationType Interpolation1D::interpolationType()
  {
    return interp_type_;
  }

  std::string Interpolation1D::interpolationTypeString() const
  {
    if (interp_type_ == ZERO_ORDER_HOLD)
    {
      return "zero_order_hold";
    }
    else if (interp_type_ == LINEAR)
    {
      return "linear";
    }
    else
    {
      return "<unknown>";
    }
  }

  void Interpolation1D::setInterpolationType(InterpolationType type)
  {
    interp_type_ = type;
  }

  bool Interpolation1D::readFromParameter(const std::string& param_name)
  {
    try
    {
      std::string interp_type = node_.get_parameter(param_name + "/interpolation_type").as_string();

      if (interp_type == "zero_order_hold")
      {
        setInterpolationType(ZERO_ORDER_HOLD);
      }
      else if (interp_type == "linear")
      {
        setInterpolationType(LINEAR);
      }
      else
      {
        RCLCPP_ERROR(node_.get_logger(), "Invalid interpolation type '%s' at '%s'.",
                     interp_type.c_str(), param_name.c_str());
        clear();
        return false;
      }
    }
    catch (...)
    {
      RCLCPP_INFO(node_.get_logger(), "No 'interpolation_type' found in %s. "
                                      "Defaulting to zero_order_hold.",
                  param_name.c_str());
      setInterpolationType(ZERO_ORDER_HOLD);
    }

    std::map<std::string, std::vector<double> > values;
    node_.get_parameters("value", values);

    if (values.find("x") == values.end())
    {
      RCLCPP_ERROR(node_.get_logger(), "Parameter had no x values.");
      clear();
      return false;
    }
    if (values.find("y") == values.end())
    {
      RCLCPP_ERROR(node_.get_logger(), "Parameter had no y values.");
      clear();
      return false;
    }
    if (values["x"].size() != values["y"].size())
    {
      RCLCPP_ERROR(node_.get_logger(), "Number of x and y values differ.");
      clear();
      return false;
    }

    for (int i = 0; i < values["x"].size(); i++)
    {
      if (!appendPoint(values["x"][i], values["y"][i]))
      {
        RCLCPP_ERROR(node_.get_logger(), "Failed to add point %s/values[%d].",
                     param_name.c_str(), i);
        clear();
        return false;
      }
    }

    return true;
  }

  double Interpolation1D::minX() const
  {
    if (x_.size() == 0)
    {
      return 0.0;
    }
    else
    {
      return x_.front();
    }
  }

  double Interpolation1D::maxX() const
  {
    if (x_.size() == 0)
    {
      return 0.0;
    }
    else
    {
      return x_.back();
    }
  }
}  // namespace swri_math_util
