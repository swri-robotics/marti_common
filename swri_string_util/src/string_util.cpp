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

#include <swri_string_util/string_util.h>

#include <cerrno>
#include <cstdlib>
#include <limits>

namespace swri_string_util
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
    int64_t number = strtol(string.c_str(), &end, base);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    if (number > std::numeric_limits<int32_t>::max() ||
        number < std::numeric_limits<int32_t>::min())
    {
      return false;
    }

    value = number;
    return true;
  }

  bool ToUInt32(const std::string& string, uint32_t& value, int32_t base)
  {
    if (string.empty())
    {
      return false;
    }

    char* end;
    errno = 0;
    int64_t number = strtol(string.c_str(), &end, base);

    // Check if an error occured or if there are junk characters at the end.
    if (errno != 0 || end != string.c_str() + string.length())
    {
      return false;
    }

    if (number > std::numeric_limits<uint32_t>::max() || number < 0)
    {
      return false;
    }

    value = number;
    return true;
  }
}
