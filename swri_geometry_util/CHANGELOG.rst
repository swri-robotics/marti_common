^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_geometry_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4 (2017-04-11)
------------------

0.2.3 (2016-12-09)
------------------
* Fix OpenCV dependencies for Kinetic build (`#400 <https://github.com/swri-robotics/marti_common/issues/400>`_)
* Contributors: P. J. Reed

0.2.2 (2016-12-07)
------------------

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------
* Update FindGEOS to generate linker flags correctly in Ubuntu 16.04 (`#348 <https://github.com/swri-robotics/marti_common/issues/348>`_).
  The regexes to find the link directory and library name from the
  output of geos-config were too liberal, so the library name
  regex would match on the `-linux` portion of the link directory,
  resulting in broken linker flags. This tightens up those regexes
  a bit to yield the correct library directory and name.
* Fix a typedef conflict in Ubuntu 16.04 (`#347 <https://github.com/swri-robotics/marti_common/issues/347>`_)
  Wrapping geos includes in #define statements forces geos to typedef
  int64 to int64_t so that it matches opencv's typedef.
* Add cubic spline interface for tf::Vector3 type.
* Contributors: Ed Venator, Marc Alban

0.1.5 (2016-05-13)
------------------
* Add an explicit dependency on pkg-config
* Contributors: P. J. Reed

0.1.4 (2016-05-12)
------------------
* Fixing Jade compilation issues
  1. For some reason, Eigen3 wasn't being properly detected by CMake.  I
  added a few lines that will make it try using PkgConfig if CMake
  fails.
  2. swri_image_util's geometry_util.test was being installed but should
  not have been; nothing else is in its "launch" directory, so I removed
  the whole directory from the install.
* Fixed compile error when ros-indigo-opencv3 is installed (`#307 <https://github.com/evenator/marti_common/issues/307>`_)
  * Fixed compile error when package ros-indigo-opencv3 is installed.
  swri_geometry_util uses wrong version of OpenCV when the package
  ros-indigo-opencv3 is installed. This patch fixes the issue.
  * Updated all CMakeFiles.txt to specify OpenCV version 2
  The find_package for OpenCV is now:
  ./swri_opencv_util/CMakeLists.txt:find_package(OpenCV 2 REQUIRED)
  ./swri_geometry_util/CMakeLists.txt:find_package(OpenCV 2 REQUIRED)
  ./swri_image_util/CMakeLists.txt:find_package(OpenCV 2)
  ./swri_transform_util/CMakeLists.txt:find_package(OpenCV 2 REQUIRED)
  Conflicts:
  swri_geometry_util/CMakeLists.txt
* Contributors: Kim Mathiassen, P. J. Reed

0.1.3 (2016-03-04)
------------------

0.1.2 (2016-01-06)
------------------

0.1.1 (2015-11-17)
------------------

0.1.0 (2015-09-29)
------------------
* Removes deprecated Eigen cmake module. (Issue `#245 <https://github.com/swri-robotics/marti_common/issues/245>`_)
* Contributors: Edward Venator

0.0.5 (2015-09-27)
------------------

0.0.4 (2015-09-27)
------------------
* Adds missing tf dependency to swri_geometry_util.
* Contributors: Ed Venator

0.0.3 (2015-09-26)
------------------
* Format package files
* Clean up dependencies
  Remove unneeded ones, add required ones not specified
* Contributors: Jerry Towler

0.0.2 (2015-09-25)
------------------
* Renames geometry_util package to swri_geometry_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
