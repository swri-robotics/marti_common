^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_image_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.15.2 (2022-09-01)
-------------------

2.14.2 (2020-09-09)
-------------------

2.14.1 (2020-08-18)
-------------------

2.14.0 (2020-07-15)
-------------------

2.13.7 (2020-06-26)
-------------------

2.13.6 (2020-06-17)
-------------------
* Update catkin dependencies (`#588 <https://github.com/swri-robotics/marti_common/issues/588>`_)
* Contributors: P. J. Reed

2.13.5 (2020-06-17)
-------------------

2.13.4 (2020-06-16)
-------------------
* Clean up Boost usage (`#584 <https://github.com/swri-robotics/marti_common/issues/584>`_)
* Contributors: P. J. Reed

2.13.3 (2020-06-12)
-------------------
* Use setuptools instead of distutils
* Contributors: P. J. Reed

2.13.2 (2020-06-10)
-------------------
* Support ROS Noetic (`#581 <https://github.com/swri-robotics/marti_common/issues/581>`_)
* Contributors: P. J. Reed

2.13.1 (2020-05-21)
-------------------

2.13.0 (2020-05-13)
-------------------
* Rename swri_image_util test (`#575 <https://github.com/swri-robotics/marti_common/issues/575>`_)
* Contributors: P. J. Reed

2.12.0 (2020-03-25)
-------------------

2.11.0 (2019-11-13)
-------------------

2.10.0 (2019-09-04)
-------------------

2.9.0 (2019-05-23)
------------------

2.8.0 (2019-02-06)
------------------

2.7.3 (2019-01-03)
------------------
* Fixes for image normalization. (`#536 <https://github.com/swri-robotics/marti_common/issues/536>`_)
* Contributors: Marc Alban

2.7.2 (2018-12-20)
------------------

2.7.1 (2018-12-14)
------------------
* Fix masked image normalization. (`#531 <https://github.com/swri-robotics/marti_common/issues/531>`_)
* Contributors: Marc Alban

2.7.0 (2018-12-04)
------------------

2.6.0 (2018-11-03)
------------------

2.5.0 (2018-10-12)
------------------

2.4.0 (2018-10-09)
------------------
* Add function to blend two images with alpha channels. (`#522 <https://github.com/swri-robotics/marti_common/issues/522>`_)
* Contributors: Marc Alban

2.3.0 (2018-05-25)
------------------

2.2.1 (2018-05-11)
------------------
* Initialize image before drawing on it. (`#512 <https://github.com/swri-robotics/marti_common/issues/512>`_)
* Contributors: Marc Alban

2.2.0 (2018-02-12)
------------------

2.1.0 (2018-01-26)
------------------

2.0.0 (2017-12-18)
------------------
* Add nodelet for drawing a polygon on an image. (`#500 <https://github.com/swri-robotics/marti_common/issues/500>`_)
* update to use non deprecated pluginlib macro (`#493 <https://github.com/swri-robotics/marti_common/issues/493>`_)
* Implement simple image file publisher. (`#488 <https://github.com/swri-robotics/marti_common/issues/488>`_)
* Contributors: Marc Alban, Mikael Arguedas

1.2.0 (2017-10-13)
------------------

1.1.0 (2017-08-31)
------------------
* Improving user feedback when checking parameter validity. Breaking apart parameter checks so it is clearer what the error is. Also making sure the user passes in non-negative values for RGB and gray values
* Add a crosshairs nodelet to swri_image_util (`#461 <https://github.com/pjreed/marti_common/issues/461>`_)
* Cloning OpenCV matrices to make sure values are not overwritten
* Fixing conditions which would trigger a warning on node startup
* Fixing problems caused by OpenCV matrices making shallow copies
* Adding compiler flag to correctly include file when using OpenCV 2.x
* Adding ability to recolor image using an OpenCV colormap
* Improving robustness of parameter parsing
* Improving error checking
* Adding color replacer to nodelet list
* Contributors: David Anthony, Edward Venator, Jerry Towler, Marc Alban, P. J. Reed

1.0.0 (2017-08-02)
------------------

* Create warp_image Nodelet (`#446 <https://github.com/evenator/marti_common/issues/446>`_)
  Add a nodelet to swri_image util that applies a 3x3 transformation matrix to an image using cv::warpPerspective and publishes the resulting image.
* Contributors: Edward Venator

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Enable blending with transparency mask (`#439 <https://github.com/pjreed/marti_common/issues/439>`_)
* Contributors: Jerry Towler, P. J. Reed

0.2.4 (2017-04-11)
------------------
* Image blending kinetic (`#429 <https://github.com/swri-robotics/marti_common/issues/429>`_)
* Initial commit of image blending
* Adding launch file and various bug fixes
* Making the base and top image encoding match. Lets us do things like blend a grayscale image onto a color image
* Removing file globbing from CMakeLists that made QtCreator happy
* Adding message_filters as a ROS package dependency
* Fix issue with contrast stretching when a grid cell is completely masked out.
* Contributors: Marc Alban, danthony06

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
* Contributors: Kim Mathiassen, Marc Alban

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

0.1.0 (2015-09-29)
------------------
* Removes deprecated Eigen cmake module. (Issue `#245 <https://github.com/swri-robotics/marti_common/issues/245>`_)
* Contributors: Edward Venator

0.0.14 (2017-04-11)
-------------------
* Image blending (`#426 <https://github.com/swri-robotics/marti_common/issues/426>`_)

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------
* Adds explicit dependency on pkg-config
* Contributors: P. J. Reed

0.0.10 (2016-05-12)
-------------------
* Update contrast stretch nodelet to automatically scale image mask to correct size.
* Contributors: danthony06

0.0.9 (2016-03-04)
------------------
* Properly installs the normalize_response node.
* No longer installs the geometry_util.test file.
* Contributors: P. J. Reed

0.0.8 (2016-01-06)
------------------

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------

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
