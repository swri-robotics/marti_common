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

#ifndef YAML_UTIL_YAML_UTIL_H_
#define YAML_UTIL_YAML_UTIL_H_

#include <stdint.h>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <swri_yaml_util/version.h>

#ifndef YAMLCPP_OLD_API
namespace YAML
{
  void operator >> (const YAML::Node& node, float& value);
  void operator >> (const YAML::Node& node, double& value);
  void operator >> (const YAML::Node& node, bool& value);
  void operator >> (const YAML::Node& node, int16_t& value);
  void operator >> (const YAML::Node& node, uint16_t& value);
  void operator >> (const YAML::Node& node, int32_t& value);
  void operator >> (const YAML::Node& node, uint32_t& value);
  void operator >> (const YAML::Node& node, int64_t& value);
  void operator >> (const YAML::Node& node, uint64_t& value);
  void operator >> (const YAML::Node& node, std::string& value);
}
#endif  // YAMLCPP_OLD_API

namespace swri_yaml_util
{
  bool LoadFile(const std::string& path, YAML::Node& yaml);
  bool LoadString(const std::string& input, YAML::Node& yaml);
  bool LoadMap(const std::map<std::string, std::string>& dict, YAML::Node& yaml);
  bool FindValue(const YAML::Node& node, const std::string& name);
  
  std::auto_ptr<YAML::Node> Clone(const YAML::Node& node);
  
  std::string ToString(double value, int32_t precision);
}

#endif  // YAML_UTIL_YAML_UTIL_H_
