// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <system_util/file_util.h>

namespace system_util
{
  boost::filesystem::path NaiveUncomplete(
    boost::filesystem::path const path,
    boost::filesystem::path const base)
  {
    // This implementation was derived from:
    //   https://svn.boost.org/trac/boost/ticket/1976#comment:2

    // Cache system-dependent dot, double-dot and slash strings
    const std::string _dot  = std::string(1, boost::filesystem::dot<boost::filesystem::path>::value);
    const std::string _dots = std::string(2, boost::filesystem::dot<boost::filesystem::path>::value);
    const std::string _sep = std::string(1, boost::filesystem::slash<boost::filesystem::path>::value);

    if (path == base) return _dot + _sep;

    boost::filesystem::path from_path;
    boost::filesystem::path from_base;
    boost::filesystem::path output;

    boost::filesystem::path::iterator path_it = path.begin();
    boost::filesystem::path::iterator base_it = base.begin();

    if ((path_it ==  path.end()) || (base_it == base.end()))
    {
      return "";
    }

    while (true)
    {
      if ((path_it == base.end()) || (base_it == base.end()) || (*path_it != *base_it))
      {
        for (; base_it != base.end(); ++base_it)
        {
          if (*base_it == _dot)
            continue;
          else if (*base_it == _sep)
            continue;
          output /= _dots + _sep;
        }

        boost::filesystem::path::iterator path_it_start = path_it;
        for (; path_it != path.end(); ++path_it)
        {
          if (path_it != path_it_start)
            output /= _sep;

          if (*path_it == _dot)
            continue;

          if (*path_it == _sep)
            continue;

          output /= *path_it;
        }
        break;
      }

      from_path /= boost::filesystem::path(*path_it);
      from_base /= boost::filesystem::path(*base_it);

      ++path_it, ++base_it;
    }

    return output;
  }
}
