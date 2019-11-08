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

#ifndef SYSTEM_UTIL_FILE_UTIL_H_
#define SYSTEM_UTIL_FILE_UTIL_H_

#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

namespace swri_system_util
{

  #if BOOST_FILESYSTEM_VERSION == 2
    typedef boost::filesystem::basic_filesystem_error<boost::filesystem::path> PathException;
  #else
    typedef boost::filesystem::filesystem_error PathException;
  #endif

  /**
   * Generate a relative file path between two absolute paths.
   *
   * @param[in]  path  The absolute path to convert to relative.
   * @param[in]  base  The base path.
   *
   * @returns The relative path if valid and an empty path otherwise.
   */
  boost::filesystem::path NaiveUncomplete(
    const boost::filesystem::path& path,
    const boost::filesystem::path& base);

  /**
   * Return a list of all file names within a directory (handles wildcard character "*").
   *
   * @param[in]  path  The absolute path containing the files (Add wildcard to filename).
   *
   * @returns The list of filenames of all found files
   */
  std::vector<std::string> load_all_files(const std::string& path, std::string& directory);

  /**
   * Return the list of files within a directory that match a regular
   * expression.
   *
   * @param[in]  path        Path to search from
   * @param[in]  expression  Regular expression for the filename.
   * @param[in]  max_depth   Max depth to search for files.  Use -1 for
   *                         unlimited depth.
   *
   * @return The list of files.
   */
  std::vector<std::string> Find(
      const std::string& path,
      const std::string& expression,
      int max_depth = -1);
}

#endif  // SYSTEM_UTIL_FILE_UTIL_H_
