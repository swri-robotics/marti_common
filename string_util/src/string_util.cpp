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

#include <string_util/string_util.h>

#include <cerrno>
#include <cstdlib>

namespace string_util
{
  bool ToDouble(const std::string& string, double& value)
  {
    if (string.empty())
    {
      return false;
    }
    
    char* end;
    errno = 0;
    double number = strtod(string.c_str(), &end);
    
    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }
    
    value = number;
    return true;
  }
  
  bool ToFloat(const std::string& string, float& value)
  {
    if (string.empty())
    {
      return false;
    }
    
    char* end;
    errno = 0;
    float number = strtof(string.c_str(), &end);
    
    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }
    
    value = number;
    return true;
  }
  
  bool ToInt32(const std::string& string, int32_t& value, int32_t base)
  {
    if (string.empty())
    {
      return false;
    }
    
    char* end;
    errno = 0;
    int32_t number = strtol(string.c_str(), &end, base);
    
    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }
    
    value = number;
    return true;
  }
}
