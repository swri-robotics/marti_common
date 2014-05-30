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
#include <string>

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
//		if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

		//It it is a directory then search within
		if ( boost::filesystem::is_directory(i->status()) )
		{
		  std::string path2 = i->path().string() + std::string("/") + path.substr(path.find_last_of("/\\")+1);
		  std::string directory2;
		  std::vector<std::string> matching_files = load_all_files( path2, directory2 );
		  all_matching_files.insert(all_matching_files.end(), matching_files.begin(), matching_files.end());
		}else if (boost::filesystem::is_regular_file( i->status())) // Check if a file
	    {
			boost::smatch what;
			// Skip if no match
			if( !boost::regex_match( i->leaf(), what, my_filter ) ) continue;
			// File matches, store it
			all_matching_files.push_back( i->path().string() );
	    }
		std::sort(all_matching_files.begin(), all_matching_files.end());
	}
	directory = direct;
    return all_matching_files;

  }
}
