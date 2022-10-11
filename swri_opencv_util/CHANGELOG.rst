^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_opencv_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.5.0 (2022-10-11)
------------------

3.4.2 (2022-10-10)
------------------
* Add Humble Support (`#691 <https://github.com/swri-robotics/marti_common/issues/691>`_)
  * Adding Humble support based on @shrijitsingh99 PR in https://github.com/swri-robotics/marti_common/pull/685
* Contributors: David Anthony, Shrijit Singh

3.3.2 (2020-11-24)
------------------

3.3.1 (2020-08-19)
------------------

3.3.0 (2020-07-15)
------------------
* Add GitHub Actions for CI (ROS2) (`#586 <https://github.com/swri-robotics/marti_common/issues/586>`_)
* Contributors: P. J. Reed

3.2.1 (2020-06-10)
------------------
* ROS Foxy support (`#582 <https://github.com/swri-robotics/marti_common/issues/582>`_)
* Contributors: P. J. Reed

3.2.0 (2020-05-13)
------------------

3.1.0 (2020-03-30)
------------------

3.0.5 (2020-03-10)
------------------

3.0.4 (2020-03-05)
------------------

3.0.3 (2019-11-11)
------------------

3.0.2 (2019-11-11)
------------------

3.0.1 (2019-11-11)
------------------

3.0.0 (2019-11-08)
------------------
* ROS2 Dashing conversion (`#549 <https://github.com/pjreed/marti_common/issues/549>`_)
* Contributors: P. J. Reed

2.10.0 (2019-09-04)
-------------------

2.9.0 (2019-05-23)
------------------

2.8.0 (2019-02-06)
------------------

2.7.3 (2019-01-03)
------------------

2.7.2 (2018-12-20)
------------------

2.7.1 (2018-12-14)
------------------

2.7.0 (2018-12-04)
------------------

2.6.0 (2018-11-03)
------------------

2.5.0 (2018-10-12)
------------------
* Add function for fitting a rotation on 3D point correspondences. (`#524 <https://github.com/swri-robotics/marti_common/issues/524>`_)
* Contributors: Marc Alban

2.4.0 (2018-10-09)
------------------
* Add function to blend two images with alpha channels. (`#522 <https://github.com/swri-robotics/marti_common/issues/522>`_)
* Contributors: Marc Alban

2.3.0 (2018-05-25)
------------------

2.2.1 (2018-05-11)
------------------
* Support ROS Melodic (`#514 <https://github.com/swri-robotics/marti_common/issues/514>`_)
* Contributors: P. J. Reed

2.2.0 (2018-02-12)
------------------

2.1.0 (2018-01-26)
------------------

2.0.0 (2017-12-18)
------------------
* Link in the "highgui" module for swri_opencv_util (`#506 <https://github.com/swri-robotics/marti_common/issues/506>`_)
* Contributors: P. J. Reed

1.2.0 (2017-10-13)
------------------
* Add PerpendicularPlaneWithPoint RANSAC model (`#487 <https://github.com/swri-robotics/marti_common/issues/487>`_)
* Contributors: Matthew Bries

1.1.0 (2017-08-31)
------------------
* Implement RANSAC and least squares model fitting for 3d geometry (`#479 <https://github.com/swri-robotics/marti_common/issues/479>`_)
* Add missing cv_bridge dependency. (`#480 <https://github.com/swri-robotics/marti_common/issues/480>`_)
* Contributors: Edward Venator, Marc Alban, P. J. Reed

1.0.0 (2017-08-02)
------------------

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/swri-robotics/marti_common/issues/443>`_)
* Contributors: P. J. Reed

0.2.4 (2017-04-11)
------------------

0.2.3 (2016-12-09)
------------------
* Fix OpenCV dependencies for Kinetic build (`#400 <https://github.com/swri-robotics/marti_common/issues/400>`_)
* Contributors: P. J. Reed

0.2.2 (2016-12-07)
------------------
* Migrated OpenCV to 3.1 (default in Kinetic)
* Contributors: Brian Holt

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------

0.1.5 (2016-05-13)
------------------

0.1.4 (2016-05-12)
------------------
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
* Contributors: Kim Mathiassen

0.1.3 (2016-03-04)
------------------

0.1.2 (2016-01-06)
------------------
* Mark some constructors explicit.
* Refactor RANSAC matching code to use more matrix operations.
* Fix bugs in FitRigidTransform2d.
  The main problem was that reshape was being incorrectly, causing the
  points to get shuffled around.  Once that was fixed, it was clear that
  the rotation should not be inverted.  Also added a comment to clarify
  the significance of the returned transform.
* Contributors: Elliot Johnson, Marc Alban

0.1.1 (2015-11-17)
------------------

0.1.0 (2015-09-29)
------------------

0.0.14 (2017-04-11)
-------------------

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------

0.0.9 (2016-03-04)
------------------

0.0.8 (2016-01-06)
------------------
* Marks some constructors explicit.
* Refactors RANSAC matching code to use more matrix operations.
* Fixes bugs in FitRigidTransform2d.
  The main problem was that reshape was being used incorrectly, causing the
  points to get shuffled around.  Once that was fixed, it was clear that
  the rotation should not be inverted.  Also adds a comment to clarify
  the significance of the returned transform.
* Contributors: Elliot Johnson, Marc Alban

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------
  Conflicts:
  swri_geometry_util/CMakeLists.txt
* Contributors: Kim Mathiassen

0.0.5 (2015-09-27)
------------------

0.0.4 (2015-09-27)
------------------

0.0.3 (2015-09-26)
------------------
* Clean up dependencies
  Remove unneeded ones, add required ones not specified
* Contributors: Jerry Towler

0.0.2 (2015-09-25)
------------------
* Renames opencv_util package to swri_opencv_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
