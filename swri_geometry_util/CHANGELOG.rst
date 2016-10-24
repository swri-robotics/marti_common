^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_geometry_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2016-10-23)
------------------
* Add cubic spline interface for tf::Vector3 type.
* Contributors: Marc Alban

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
