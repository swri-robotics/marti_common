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

#include <regex>
#include <string>

namespace swri_system_util
{
  std::filesystem::path NaiveUncomplete(
    const std::filesystem::path& path,
    const std::filesystem::path& base)
  {
    // Cache system-dependent dot, double-dot and slash strings

    const std::filesystem::path _dot = std::filesystem::path(".").native();
    const std::filesystem::path _dot_sep = std::filesystem::path("./").native();
    const std::filesystem::path _dots = std::filesystem::path("..").native();
    const std::filesystem::path _dots_sep = std::filesystem::path("../").native();
    const std::filesystem::path _sep = std::filesystem::path("/").native();

    if (path == base) return _dot_sep;

    std::filesystem::path from_path;
    std::filesystem::path from_base;
    std::filesystem::path output;

    auto path_it = path.begin();
    auto base_it = base.begin();

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
          {
            continue;
          }
          else if (*base_it == _sep)
          {
            continue;
          }
          output /= _dots_sep;
        }

        auto path_it_start = path_it;
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

      from_path /= std::filesystem::path(*path_it);
      from_base /= std::filesystem::path(*base_it);

      ++path_it;
      ++base_it;
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
    std::filesystem::path p(direct);
    if (!exists( p ))
    {
      printf("Path %s does not exists\n", p.string().c_str());
      return all_matching_files;
    }
    const std::regex my_filter(filename.replace(filename.find("*"), std::string("*").length(), ".*\\") );

    std::filesystem::directory_iterator end_itr; // Default construction yields past-the-end
    for (std::filesystem::directory_iterator i(p); i != end_itr; ++i )
    {
      //It it is a directory then search within
      if ( std::filesystem::is_directory(i->status()) )
      {
        std::string path2 = i->path().string() + std::string("/") + path.substr(path.find_last_of("/\\")+1);
        std::string directory2;
        std::vector<std::string> matching_files = load_all_files( path2, directory2 );
        all_matching_files.insert(all_matching_files.end(), matching_files.begin(), matching_files.end());
      }
      else if (std::filesystem::is_regular_file( i->status())) // Check if a file
      {
        std::smatch what;
        // Skip if no match. Need a temporary string for GCC quirk
        std::string temp_string = i->path().filename().string();
        if( !std::regex_match(temp_string, what, my_filter ) ) continue;

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

    std::filesystem::path root(path);
    if(!std::filesystem::exists(root) || !std::filesystem::is_directory(root))
    {
      return files;
    }

    std::regex filter(expression);
    std::filesystem::recursive_directory_iterator it(root);
    std::filesystem::recursive_directory_iterator end_it;
    while (it != end_it)
    {
      if (max_depth >= 0 && it.depth() >= max_depth)
      {
        it.disable_recursion_pending();
      }

      std::smatch what;
      std::string filename = it->path().filename().string();
      if (std::regex_match(filename, what, filter))
      {
        files.push_back(it->path().string());
      }

      ++it;
    }

    return files;
  }
}
