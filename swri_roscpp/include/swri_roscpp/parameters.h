// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_PARAMETERS_H_
#define SWRI_ROSCPP_PARAMETERS_H_

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include <boost/bind.hpp>

#include <rclcpp/rclcpp.hpp>

namespace swri
{
  static inline std::string clean(const std::string& name)
  {
    std::string clean = name;

    size_t pos = clean.find("//");
    while (pos != std::string::npos)
    {
      clean.erase(pos, 1);
      pos = clean.find("//", pos);
    }

    if (*clean.rbegin() == '/')
    {
      clean.erase(clean.size() - 1, 1);
    }

    return clean;
  }

  static inline std::string append(const std::string& left, const std::string& right)
  {
    return clean(left + "/" + right);
  }

  static inline
  bool getParam(const rclcpp::Node& nh,
                const std::string& name,
                int& variable)
  {
    if (!nh.get_parameter(name, variable))
    {
      RCLCPP_ERROR(nh.get_logger(), "Required int parameter %s does not exist", name.c_str());
      return false;
    }
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %d", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const rclcpp::Node& nh,
                const std::string& name,
                double& variable)
  {
    if (!nh.get_parameter(name, variable))
    {
      RCLCPP_ERROR(nh.get_logger(), "Required double parameter %s does not exist", name.c_str());
      return false;
    }
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %lf", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const rclcpp::Node& nh,
                const std::string& name,
                float& variable)
  {
    double dbl_value;
    if (!nh.get_parameter(name, dbl_value))
    {
      RCLCPP_ERROR(nh.get_logger(), "Required double parameter %s does not exist", name.c_str());
      return false;
    }
    variable = dbl_value;
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %f", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const rclcpp::Node& nh,
                const std::string& name,
                std::string& variable)
  {
    if (!nh.get_parameter(name, variable))
    {
      RCLCPP_ERROR(nh.get_logger(), "Required string parameter %s does not exist", name.c_str());
      return false;
    }
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %s", name.c_str(), variable.c_str());
    return true;
  }

  static inline
  bool getParam(const rclcpp::Node& nh,
                const std::string& name,
                bool& variable)
  {
    if (!nh.get_parameter(name, variable))
    {
      RCLCPP_ERROR(nh.get_logger(), "Required bool parameter %s does not exist", name.c_str());
      return false;
    }
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
    return true;
  }

  static inline
  void param(const rclcpp::Node& nh,
             const std::string& name,
             int& variable,
             const int default_value)
  {
    nh.get_parameter_or(name, variable, default_value);
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %d", name.c_str(), variable);
  }

  static inline
  void param(const rclcpp::Node& nh,
             const std::string& name,
             double& variable,
             const double default_value)
  {
    nh.get_parameter_or(name, variable, default_value);
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %lf", name.c_str(), variable);
  }

  static inline
  void param(const rclcpp::Node& nh,
             const std::string& name,
             float& variable,
             const float default_value)
  {
    double dbl_value;
    double dbl_default = default_value;
    nh.get_parameter_or(name, dbl_value, dbl_default);
    variable = dbl_value;
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %f", name.c_str(), variable);
  }

  static inline
  void param(const rclcpp::Node& nh,
             const std::string& name,
             std::string& variable,
             const std::string default_value)
  {
    nh.get_parameter_or(name, variable, default_value);
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = \"%s\"", name.c_str(), variable.c_str());
  }

  static inline
  void param(const rclcpp::Node& nh,
             const std::string& name,
             bool& variable,
             const bool default_value)
  {
    nh.get_parameter_or(name, variable, default_value);
    RCLCPP_INFO(nh.get_logger(), "Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
  }

  /**
   * Determines whether `prefix` is a prefix of `string`.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return 0 if the first n characters of `string` are identical to `prefix`,
   *         where n is the length of `prefix`. 1 or -1 otherwise, according to
   *         the conventions of std::string::compare()
   */
  inline int comparePrefix(std::string const& string, std::string const& prefix)
  {
    return string.compare(0, prefix.size(), prefix);
  }

  /**
   * Boolean wrapper around comparePrefix.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return True if the first n characters of `string` are identical to
   *         `prefix`, where n is the length of `prefix`, false otherwise.
   */
  inline bool isPrefixOf(std::string const& string, std::string const& prefix)
  {
    return comparePrefix(string, prefix) == 0;
  }

  /**
   * Less-than wrapper around comparePrefix, for algorithms that require a
   * less-than comparator, such as sorting.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return True if the first n characters of `string` are "less than" the
   *         first n to `prefix`, where n is the length of `prefix`, according
   *         to the conventions of std::string::compare(). False otherwise.
   */
  inline int prefixLessThan(std::string const& string, std::string const& prefix)
  {
    return comparePrefix(string, prefix) < 0;
  }
}  // namespace swri_param
#endif  // SWRI_ROSCPP_PARAMETERS_H_
