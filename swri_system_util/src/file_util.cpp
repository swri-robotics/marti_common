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

#include <swri_system_util/file_util.h>

#include <string>

#include <boost/regex.hpp>

namespace swri_system_util
{
  boost::filesystem::path NaiveUncomplete(
    const boost::filesystem::path& path,
    const boost::filesystem::path& base)
  {
    // This implementation was derived from:
    //   https://svn.boost.org/trac/boost/ticket/1976#comment:2

    // Cache system-dependent dot, double-dot and slash strings

    const boost::filesystem::path _dot = boost::filesystem::path(".").native();
    const boost::filesystem::path _dot_sep = boost::filesystem::path("./").native();
    const boost::filesystem::path _dots = boost::filesystem::path("..").native();
    const boost::filesystem::path _dots_sep = boost::filesystem::path("../").native();
    const boost::filesystem::path _sep = boost::filesystem::path("/").native();

    if (path == base) return _dot_sep;

    boost::filesystem::path from_path;
    boost::filesystem::path from_base;
    boost::filesystem::path output;

    boost::filesystem::path::iterator path_it = path.begin();
    boost::filesystem::path::iterator base_it = base.begin();

    if ((path_it == path.end()) || (base_it == base.end()))
    {
      return "";
    }

    while (true)
    {
      if (*path_it != *base_it)
      {
        for (; base_it != base.end(); ++base_it)
        {
          if (*base_it == _dot)
            continue;
          else if (*base_it == _sep)
            continue;
          output /= _dots_sep;
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

  std::vector<std::string> load_all_files(const std::string& path, std::string& directory)
  {
    std::vector< std::string > all_matching_files;
    // Extract the directory from the path
    std::string direct = path.substr(0, path.find_last_of("/\\"));
    // Extract the filename from the path
    std::string filename = path.substr(path.find_last_of("/\\")+1);
    boost::filesystem::path p(direct);
    if ( !exists( p ) )
    {
      printf("Path %s does not exists\n", p.string().c_str());
      return all_matching_files;
    }
    const boost::regex my_filter(filename.replace(filename.find("*"), std::string("*").length(), ".*\\") );

    boost::filesystem::directory_iterator end_itr; // Default construction yields past-the-end
    for( boost::filesystem::directory_iterator i( p ); i != end_itr; ++i )
    {
      // Skip if not a file
      // if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

      //It it is a directory then search within
      if ( boost::filesystem::is_directory(i->status()) )
      {
        std::string path2 = i->path().string() + std::string("/") + path.substr(path.find_last_of("/\\")+1);
        std::string directory2;
        std::vector<std::string> matching_files = load_all_files( path2, directory2 );
        all_matching_files.insert(all_matching_files.end(), matching_files.begin(), matching_files.end());
      }
      else if (boost::filesystem::is_regular_file( i->status())) // Check if a file
      {
        boost::smatch what;
        // Skip if no match
        if( !boost::regex_match( i->path().filename().string(), what, my_filter ) ) continue;

        // File matches, store it
        all_matching_files.push_back( i->path().string() );
      }
      std::sort(all_matching_files.begin(), all_matching_files.end());
    }
    directory = direct;
    return all_matching_files;
  }

  std::vector<std::string> Find(
      const std::string& path,
      const std::string& expression,
      int max_depth)
  {
    std::vector<std::string> files;

    boost::filesystem::path root(path);
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    {
      return files;
    }

    boost::regex filter(expression);
    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator end_it;
    while (it != end_it)
    {
      if (max_depth >= 0 && it.level() >= max_depth)
      {
        it.no_push();
      }

      boost::smatch what;
      std::string filename = it->path().filename().string();
      if (boost::regex_match(filename, what, filter))
      {
        files.push_back(it->path().string());
      }

      ++it;
    }

    return files;
  }
}
