// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#ifndef YAML_UTIL_YAML_UTIL_H_
#define YAML_UTIL_YAML_UTIL_H_

#include <stdint.h>

#include <yaml-cpp/yaml.h>

#include <yaml_util/version.h>

#ifndef YAMLCPP_OLD_API
namespace YAML
{
  void operator >> (const YAML::Node& node, float& value);
  void operator >> (const YAML::Node& node, double& value);
  void operator >> (const YAML::Node& node, bool& value);
  void operator >> (const YAML::Node& node, int16_t& value);
  void operator >> (const YAML::Node& node, int32_t& value);
  void operator >> (const YAML::Node& node, uint32_t& value);
  void operator >> (const YAML::Node& node, int64_t& value);
  void operator >> (const YAML::Node& node, uint64_t& value);
  void operator >> (const YAML::Node& node, std::string& value);
}
#endif  // YAMLCPP_OLD_API

namespace yaml_util
{
  bool LoadFile(const std::string& path, YAML::Node& yaml);
  bool FindValue(const YAML::Node& node, const std::string& name);
}

#endif  // YAML_UTIL_YAML_UTIL_H_
