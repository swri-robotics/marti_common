^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_transform_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#435 <https://github.com/swri-robotics/marti_common/issues/435>`_ from swri-robotics/initialize-origin-license
  Fix whitespace and license in initialize_origin.py
* Fix whitespace and license in initialize_origin.py
  Replace "all rights reserved" with standard BSD 3-clause text and remove trailing whitespace in initialize_origin.py
* Fixes `#431 <https://github.com/swri-robotics/marti_common/issues/431>`_
* Simplify dynamic reconfigure usage.
* Add nodelet for publishing a dynamically reconfigurable TF transform.
* Contributors: Edward Venator, Marc Alban, P. J. Reed

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------
* Add explicit getOrientation function for Utm transformer
* Improve georeferencing warnings.
* Contributors: Jason Gassaway, Marc Alban

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------
* Add great circle distance method for tf::Vector3 type.
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

0.0.9 (2016-03-04)
------------------
* Fixes initialize_origin.py diagnostic reporting a warning that the
  origin is not automatic when it is.
* Adds transform publisher to initialize_origin.py that publishes an
  identity transform from the local_xy_frame to an anonymous unused
  frame.  In doing so, the local_xy_frame will show up
  in the /tf tree without any additional nodes running so that
  TransformManager can properly transform between /wgs84 and /map.
  This change should not interfere with any existing systems.
* Expands some of the TransformManager warnings to be more
  informative.  This is to reduce the impact of common problems that we
  run into when setting up a new environment by making it easier to
  distinguish the exact nature of the error, as well as provide
  suggestions when appropriate.
  In particular, this fixes the misleading
  "No transfomer from /wgs84 to /map" error and upgrades a warning
  about null pointers to an error.
* Contributors: Elliot Johnson

0.0.8 (2016-01-06)
------------------
* Accounts for non-zero reference angles when calculating orientations to and from WGS84.
* Publishes origin with east orientation (0 yaw) by default.
* Supports arbitrary local_xy reference angles.
  * The reference heading is renamed to reference angle.
  * It's not recommended to set a non-zero reference angle.
  * Adds a parameter to ignore the reference heading for backwards compatibility.
* Fixes backwards compatibility issue with swri_yaml_cpp call.
* Contributors: Kris Kozak, Marc Alban

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------
* Adds a GetTF method to transform_util::Transform.
* Properly installs the initialize_origin.py node.
* Add extension type (e.g. png) in geo file
* Contributors: Edward Venator, P. J. Reed, Vincent Rousseau

0.0.5 (2015-09-27)
------------------

0.0.4 (2015-09-27)
------------------
* Fixes missing dependencies. `#239 <https://github.com/swri-robotics/marti_common/issues/239>`_.
* Contributors: Ed Venator

0.0.3 (2015-09-26)
------------------

0.0.2 (2015-09-25)
------------------
* Renames yaml_util to swri_yaml_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Renames transform_util to swri_transform_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
