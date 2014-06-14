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

#ifndef SYSTEM_UTIL_FILE_UTIL_H_
#define SYSTEM_UTIL_FILE_UTIL_H_

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

namespace system_util
{
  /**
   * Generate a relative file path between two absolute paths.
   *
   * @param[in]  path  The absolute path to convert to relative.
   * @param[in]  base  The base path.
   *
   * @returns The relative path if valid and an empty path otherwise.
   */
  boost::filesystem::path NaiveUncomplete(
    boost::filesystem::path const path,
    boost::filesystem::path const base);
}

#endif  // SYSTEM_UTIL_FILE_UTIL_H_
