^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_image_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4 (2017-04-11)
------------------
* Image blending kinetic (`#429 <https://github.com/swri-robotics/marti_common/issues/429>`_)
  * Initial commit of image blending
  * Adding launch file and various bug fixes
  * Making the base and top image encoding match. Lets us do things like blend a grayscale image onto a color image
  * Removing file globbing from CMakeLists that made QtCreator happy
  * Adding message_filters as a ROS package dependency
* Contributors: danthony06

0.2.3 (2016-12-09)
------------------
* Fix OpenCV dependencies for Kinetic build (`#400 <https://github.com/swri-robotics/marti_common/issues/400>`_)
* Contributors: P. J. Reed

0.2.2 (2016-12-07)
------------------
* Fix issue with contrast stretching when a grid cell is completely masked out.
* Migrated OpenCV to 3.1 (default in Kinetic)
* Contributors: Brian Holt, Marc Alban

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------
* Replace legacy OpenCV BruteForceMatcher with new cv::BFMatcher.
* Upgrade Qt to version 5.
* Contributors: Ed Venator

0.1.5 (2016-05-13)
------------------
* Add an explicit dependency on pkg-config
* Contributors: P. J. Reed

0.1.4 (2016-05-12)
------------------
* Update contrast stretch nodelet to automatically scale image mask to correct size.
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
* Contributors: Kim Mathiassen, Marc Alban, P. J. Reed

0.1.3 (2016-03-04)
------------------

0.1.2 (2016-01-06)
------------------
* Fixes nodelet description for normalize_response.
* Tweaks contrast stretching to increase blending of min/max bounds across grid.
* Removes some C-style casts.
* Adds parameters for masking out over exposed areas out of the contrast stretch processing.
* Adds normalize response image normalization method.
* Contributors: Marc Alban

0.1.1 (2015-11-17)
------------------
* Image normalization now supports normalization to a min/max range.
* Contributors: Marc Alban

0.1.0 (2015-09-29)
------------------
* Removes deprecated Eigen cmake module. (Issue `#245 <https://github.com/swri-robotics/marti_common/issues/245>`_)
* Contributors: Edward Venator

0.0.5 (2015-09-27)
------------------

0.0.4 (2015-09-27)
------------------

0.0.3 (2015-09-26)
------------------
* Fixes missing depend on swri_opencv_util in swri_image_util.
* Clean up dependencies
  Remove unneeded ones, add required ones not specified
* Contributors: Ed Venator, Jerry Towler

0.0.2 (2015-09-25)
------------------
* Renames opencv_util package to swri_opencv_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_
* Renames math_util to swri_math_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Renames image_util package to swri_image_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
