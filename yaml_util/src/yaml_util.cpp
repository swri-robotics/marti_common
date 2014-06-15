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

#include <yaml_util/yaml_util.h>

// C++ standard libraries
#ifdef YAMLCPP_OLD_API
#include <fstream>
#endif  // YAMLCPP_OLD_API

#ifndef YAMLCPP_OLD_API
namespace YAML
{
  void operator >> (const YAML::Node& node, float& value)
  {
    value = node.as<float>();
  }

  void operator >> (const YAML::Node& node, double& value)
  {
    value = node.as<double>();
  }

  void operator >> (const YAML::Node& node, bool& value)
  {
    value = node.as<bool>();
  }

  void operator >> (const YAML::Node& node, int16_t& value)
  {
    value = node.as<int16_t>();
  }

  void operator >> (const YAML::Node& node, int32_t& value)
  {
    value = node.as<int32_t>();
  }

  void operator >> (const YAML::Node& node, uint32_t& value)
  {
    value = node.as<uint32_t>();
  }

  void operator >> (const YAML::Node& node, int64_t& value)
  {
    value = node.as<int64_t>();
  }
  
  void operator >> (const YAML::Node& node, uint64_t& value)
  {
    value = node.as<uint64_t>();
  }

  void operator >> (const YAML::Node& node, std::string& value)
  {
    value = node.as<std::string>();
  }
}
#endif  // YAMLCPP_OLD_API

namespace yaml_util
{
  bool LoadFile(const std::string& path, YAML::Node& yaml)
  {
    try
    {
      #ifndef YAMLCPP_OLD_API
      yaml = YAML::LoadFile(path);
      #else
      std::ifstream fin(path.c_str());
      if (fin.fail())
      {
        return false;
      }
      
      YAML::Parser parser(fin);
      parser.GetNextDocument(yaml);
      #endif  // YAMLCPP_OLD_API
    }
    catch (...)
    {
      return false;
    }
    
    return true;
  }
  
  bool FindValue(const YAML::Node& node, const std::string& name)
  {
    #ifndef YAMLCPP_OLD_API
      return node[name];
    #else
      return node.FindValue(name);
    #endif  // YAMLCPP_OLD_API
  }
}
