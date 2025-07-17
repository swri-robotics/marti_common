^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_transform_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.8.6 (2025-07-17)
------------------
* Switch to using yaml_cpp_vendor (`#784 <https://github.com/swri-robotics/marti_common/issues/784>`_)
  * switch to yaml_cpp_vendor to ensure that the version that is used is the same as is used in core ros packages
  ---------
  Co-authored-by: Mark B. Allan <Mark.B.Allan@nasa.gov>
  Co-authored-by: David Anthony <djanthony@gmail.com>
* Contributors: Mark B. Allan

3.8.5 (2025-06-25)
------------------
* Improved GeographicLib Package Finding
* Contributors: DangitBen

3.8.4 (2025-06-10)
------------------
* Add dependency on libopencv-dev for pkgs directly using OpenCV (`#781 <https://github.com/swri-robotics/marti_common/issues/781>`_)
* Updating setup.py to remove deprecated features (`#779 <https://github.com/swri-robotics/marti_common/issues/779>`_)
* Contributors: Christophe Bedard, David Anthony

3.8.3 (2025-06-03)
------------------

3.8.2 (2025-06-02)
------------------
* Dependency cleanup (`#774 <https://github.com/swri-robotics/marti_common/issues/774>`_)
* Contributors: DangitBen

3.8.1 (2025-05-29)
------------------
* modernize cmake targets and exporting (`#772 <https://github.com/swri-robotics/marti_common/issues/772>`_)
* Contributors: David Anthony, Ben Andrew

3.8.0 (2025-05-27)
------------------
* Updates for Rolling (`#771 <https://github.com/swri-robotics/marti_common/issues/771>`_)
  * Removed obsolete ament macros
  * Removed boost in favor of standard library functionality
  * Deprecated swri_system_util
* Contributors: David Anthony

3.7.6 (2025-05-22)
------------------
* Reverting changes back to 3.7.4 (`#770 <https://github.com/swri-robotics/marti_common/issues/770>`_)
* Contributors: David Anthony

3.7.5 (2025-05-20)
------------------
* Rolling Updates (`#767 <https://github.com/swri-robotics/marti_common/issues/767>`_)
  * Updating with new header file names
  * Removing deprecated ament calls
  ---------
  Co-authored-by: David V. Lu!! <davidvlu@gmail.com>
* [kilted] Update deprecated calls to ament_target_dependencies (`#763 <https://github.com/swri-robotics/marti_common/issues/763>`_)
* Update local_xy_util.cpp (`#764 <https://github.com/swri-robotics/marti_common/issues/764>`_)
  Changing include to new .hpp file.
* Contributors: David Anthony, David V. Lu!!

3.7.4 (2025-04-14)
------------------

3.7.3 (2024-09-18)
------------------
* Update origin params (`#752 <https://github.com/swri-robotics/marti_common/issues/752>`_)
  * Update origin publisher params
  ---------
  Co-authored-by: Ben <benjamin.andrew@swri.org>
* Contributors: DangitBen

3.7.2 (2024-09-16)
------------------

3.7.1 (2024-09-06)
------------------
* Remove transforms3d as dependency because of packaging problems (`#746 <https://github.com/swri-robotics/marti_common/issues/746>`_)
* Contributors: David Anthony

3.7.0 (2024-09-06)
------------------
* Fix Test Failures (`#745 <https://github.com/swri-robotics/marti_common/issues/745>`_)
  * Checking if increasing timeout value will fix test failure
  * Making default origin all NaN
* Updating Tests With New Error Bounds (`#741 <https://github.com/swri-robotics/marti_common/issues/741>`_)
* Replaced local projection math with geographiclib (`#739 <https://github.com/swri-robotics/marti_common/issues/739>`_)
  * Replaced local projection math with geographiclib
  * Updated unit tests with new values
* Fix Unit Test Failures (`#740 <https://github.com/swri-robotics/marti_common/issues/740>`_)
  * Fixing inverse transform
  * Fixing various CI build failures.
* changed from python-transforms3d-pip dep to python3-transforms3d dep. (`#730 <https://github.com/swri-robotics/marti_common/issues/730>`_)
* Fix UTM Utils and Get Tests to Build in ROS 2 (`#729 <https://github.com/swri-robotics/marti_common/issues/729>`_)
  * Fix north and south projections
  * Enable tests and fix coordinate types
  * Updating test condition to match ROS 1 condition
  * Format build files like on other in-progress branch
  * Got a couple of the swri_transform_util unit tests working.
  * Got a few more transform manager tests to work. Loss of precision somewhere causing existing test to fail.
  * Uncommented all swri_transform_util::TransformManager tests.
  * Update tests for ROS 2 except pytests
  * Port python tests
  * Update navsatfix test to match gpsfix test
  ---------
  Co-authored-by: robert.brothers <robert.brothers@swri.org>
* Adding better initialization of origin (`#725 <https://github.com/swri-robotics/marti_common/issues/725>`_)
* Adding Adelaide UTM <-> WGS84 test case (`#723 <https://github.com/swri-robotics/marti_common/issues/723>`_)
* Fixing wrapping around negative/positive lat/lon changes (`#722 <https://github.com/swri-robotics/marti_common/issues/722>`_)
* Cleaning up package maintainer (`#721 <https://github.com/swri-robotics/marti_common/issues/721>`_)
* Contributors: David Anthony, JayHerpin, Robert Brothers, Veronica Knisley

3.6.1 (2023-09-11)
------------------
* Fixing Buildfarm Issues (`#716 <https://github.com/swri-robotics/marti_common/issues/716>`_)
  * Cleaning up code to fix build farm errors
* Contributors: David Anthony

3.6.0 (2023-08-22)
------------------
* Rolling updates (`#715 <https://github.com/swri-robotics/marti_common/issues/715>`_)
  - Updating code for Rolling.
  - Removing conditional compilation code related to EOL distributions.
* Contributors: David Anthony

3.5.4 (2023-08-14)
------------------

3.5.3 (2023-06-07)
------------------
* Fixing build error in Iron (`#709 <https://github.com/swri-robotics/marti_common/issues/709>`_)
* Contributors: David Anthony

3.5.2 (2023-05-30)
------------------
* Fix dependencies (`#704 <https://github.com/danthony06/marti_common/issues/704>`_)
  * Fixing build process so dependencies are correctly exported.
* Make timeout 0.1 seconds instead of 0 (`#694 <https://github.com/danthony06/marti_common/issues/694>`_)
* Set origin (`#696 <https://github.com/danthony06/marti_common/issues/696>`_)
  * Improved origin initialization
  ---------
  Co-authored-by: tcappellari <tiffany.cappellari@swri.org>
* Contributors: David Anthony, Tiffany Cappellari, Veronica Knisley

3.5.1 (2022-11-29)
------------------

3.5.0 (2022-10-11)
------------------
* Fix ament exports (`#693 <https://github.com/swri-robotics/marti_common/issues/693>`_)
  * Fix ament exports
* Contributors: P. J. Reed

3.4.2 (2022-10-10)
------------------
* Add Humble Support (`#691 <https://github.com/swri-robotics/marti_common/issues/691>`_)
  * Adding Humble support based on @shrijitsingh99 PR in https://github.com/swri-robotics/marti_common/pull/685
* Contributors: David Anthony, Shrijit Singh

3.3.2 (2020-11-24)
------------------

3.3.1 (2020-08-19)
------------------
* Fix bad conversion when 90 degree yaw specified (`#599 <https://github.com/swri-robotics/marti_common/issues/599>`_)
* Contributors: Alex Youngs

3.3.0 (2020-07-15)
------------------
* Add python wgs84 transformer ros2 (`#596 <https://github.com/swri-robotics/marti_common/issues/596>`_)
* Improve object transformer to take into account the pose of the object (`#589 <https://github.com/swri-robotics/marti_common/issues/589>`_) (`#591 <https://github.com/swri-robotics/marti_common/issues/591>`_)
* Add GitHub Actions for CI (ROS2) (`#586 <https://github.com/swri-robotics/marti_common/issues/586>`_)
* Contributors: P. J. Reed, Alex Youngs, Matthew Bries

3.2.1 (2020-06-10)
------------------
* ROS Foxy support (`#582 <https://github.com/swri-robotics/marti_common/issues/582>`_)
* Contributors: P. J. Reed

3.2.0 (2020-05-13)
------------------

3.1.0 (2020-03-30)
------------------
* Fix swri_transform_util::TransformManager initialization issues
* Implement initialize_origin.py in ROS Eloquent
* Contributors: P. J. Reed

3.0.5 (2020-03-10)
------------------
* Also add NavSatFix support to swri_transform_util::LocalXyUtil (`#569 <https://github.com/swri-robotics/marti_common/issues/569>`_)
* Contributors: P. J. Reed

3.0.4 (2020-03-05)
------------------
* ROS 2 Eloquent compatibility (`#568 <https://github.com/swri-robotics/marti_common/issues/568>`_)
* Replace boost::array with std::array (`#567 <https://github.com/swri-robotics/marti_common/issues/567>`_)
* Fix a crash that happened due to an initialization error (`#566 <https://github.com/swri-robotics/marti_common/issues/566>`_)
* Fix TransformManager so it works in ROS2 (`#565 <https://github.com/swri-robotics/marti_common/issues/565>`_)
* Port ObstacleTransformer node to ROS2 (`#559 <https://github.com/swri-robotics/marti_common/issues/559>`_)
* Remove "nodelets" directory (`#558 <https://github.com/swri-robotics/marti_common/issues/558>`_)
* Contributors: P. J. Reed

3.0.3 (2019-11-11)
------------------

3.0.2 (2019-11-11)
------------------
* Comment out ament_cmake_gtest (`#555 <https://github.com/pjreed/marti_common/issues/555>`_)
* Contributors: P. J. Reed

3.0.1 (2019-11-11)
------------------
* Add missing dependency on Boost::thread to swri_transform_util (`#554 <https://github.com/pjreed/marti_common/issues/554>`_)
* Contributors: P. J. Reed

3.0.0 (2019-11-08)
------------------
* ROS2 Dashing conversion (`#549 <https://github.com/pjreed/marti_common/issues/549>`_)
* Contributors: P. J. Reed

2.10.0 (2019-09-04)
-------------------
* Support in initialize_origin.py specifying custom ROS topic in parameter (`#544 <https://github.com/swri-robotics/marti_common/issues/544>`_)
* Fix get relative transform (`#546 <https://github.com/swri-robotics/marti_common/issues/546>`_)
* Contributors: Arkady Shapkin, Matthew

2.9.0 (2019-05-23)
------------------

2.8.0 (2019-02-06)
------------------
* Reduce the number of local_xy subscribers from 3 per transform manager to 1 (`#537 <https://github.com/swri-robotics/marti_common/issues/537>`_)
* Add dynamic parameters (`#532 <https://github.com/swri-robotics/marti_common/issues/532>`_)
* Contributors: Matthew

2.7.3 (2019-01-03)
------------------

2.7.2 (2018-12-20)
------------------
* Fix the unit test that intermittently breaks (`#534 <https://github.com/swri-robotics/marti_common/issues/534>`_)
* Contributors: P. J. Reed

2.7.1 (2018-12-14)
------------------

2.7.0 (2018-12-04)
------------------

2.6.0 (2018-11-03)
------------------

2.5.0 (2018-10-12)
------------------

2.4.0 (2018-10-09)
------------------
* Handle invalid fixes properly (`#519 <https://github.com/swri-robotics/marti_common/issues/519>`_)
* Contributors: P. J. Reed

2.3.0 (2018-05-25)
------------------
* Initialize transform timestamp to 0 instead of ros::Time::now() (`#515 <https://github.com/swri-robotics/marti_common/issues/515>`_)
* Contributors: Marc Alban

2.2.1 (2018-05-11)
------------------
* Support ROS Melodic (`#514 <https://github.com/swri-robotics/marti_common/issues/514>`_)
* Contributors: P. J. Reed

2.2.0 (2018-02-12)
------------------
* Add gps_transform_publisher. (`#509 <https://github.com/swri-robotics/marti_common/issues/509>`_)
* Contributors: Marc Alban

2.1.0 (2018-01-26)
------------------

2.0.0 (2017-12-18)
------------------
* expose TransformManager::LocalXyUtil() and LocalXyWgs84Util::ResetInitialization() (`#501 <https://github.com/swri-robotics/marti_common/issues/501>`_)
* Complete rewrite of initialize_origin.py (`#491 <https://github.com/swri-robotics/marti_common/issues/491>`_)
* Normalize TF frames before comparisons. (`#492 <https://github.com/swri-robotics/marti_common/issues/492>`_)
* Add new methods that expose the frame timeout. (`#498 <https://github.com/swri-robotics/marti_common/issues/498>`_)
* Use pkgconfig to include libproj in swri_transform_util
* Contributors: Davide Faconti, Edward Venator, P. J. Reed

1.2.0 (2017-10-13)
------------------

1.1.0 (2017-08-31)
------------------
* Revert "Remove nodelet_plugins.xml from CMakeLists.txt" (`#475 <https://github.com/pjreed/marti_common/issues/475>`_)
* Document swri_transform_util (`#456 <https://github.com/pjreed/marti_common/issues/456>`_)
* Contributors: Edward Venator, Marc Alban, P. J. Reed

1.0.0 (2017-08-02)
------------------
* Increase delay before running tests.
* Integrate transformers as static classes instead of plug-ins.
* Add inverse transform implementation to transforms. (`#464 <https://github.com/evenator/marti_common/issues/464>`_)
* Add tests for initialize_origin.py script (`#457 <https://github.com/evenator/marti_common/issues/457>`_)
* Contributors: Edward Venator, Marc Alban

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Fix dynamic reconfigure in dynamic_publisher (closes issue `#448 <https://github.com/pjreed/marti_common/issues/448>`_).
* Contributors: Elliot Johnson, P. J. Reed

0.2.4 (2017-04-11)
------------------
* Ignore invalid fixes
  Fixes `#431 <https://github.com/swri-robotics/marti_common/issues/431>`_.
* Remove unused gps_common dependency (`#422 <https://github.com/swri-robotics/marti_common/issues/422>`_)
  Fix `#421 <https://github.com/swri-robotics/marti_common/issues/421>`_ by removing gps_common from the swri_transform_util CMakeLists.txt in kinetic.
* Simplify dynamic reconfigure usage.
* Add nodelet for publishing a dynamically reconfigurable TF transform.
* Contributors: Edward Venator, Marc Alban, P. J. Reed

0.2.3 (2016-12-09)
------------------

0.2.2 (2016-12-07)
------------------
* Migrated OpenCV to 3.1 (default in Kinetic)
* Contributors: Brian Holt

0.2.1 (2016-10-23)
------------------
* Improve georeferencing warnings.
* Contributors: Marc Alban

0.2.0 (2016-06-21)
------------------

0.1.5 (2016-05-13)
------------------

0.1.4 (2016-05-12)
------------------
* Add great circle distance method for tf::Vector3 type.
* Fixed compile error when ros-indigo-opencv3 is installed (`#307 <https://github.com/evenator/marti_common/issues/307>`_)
* Contributors: Kim Mathiassen, Marc Alban

0.1.3 (2016-03-04)
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

0.1.2 (2016-01-06)
------------------
* Account for non-zero reference angles when calculating orientations to and from WGS84.
* Support arbitrary local_xy reference angles.
  * The reference heading has been renamed to reference angle.
  * It's not recommended to set a non-zero reference angle.
  * A parameter is provided to ignore the reference heading for backwards compatibility.
* Fix backwards compatibility issue with swri_yaml_cpp call.
* Contributors: Kris Kozak, Marc Alban

0.1.1 (2015-11-17)
------------------
* Adds a GetTF method to transform_util::Transform.
* Installing the initialize_origin.py node.
* Add extension type (e.g. png) in geo file
* Contributors: Edward Venator, P. J. Reed, Vincent Rousseau

0.1.0 (2015-09-29)
------------------
* Updates lot_lon_tf_echo to use geometry_msgs/PoseStamped.
  See issue `#246 <https://github.com/evenator/marti_common/issues/246>`__
* Removes dependency on gps_common
  The gps_common package was removed in ROS Jade, so a different message
  type is needed for the local XY origin message. (Issue `#246 <https://github.com/swri-robotics/marti_common/issues/246>`__).
  This replaces the gps_common/GPSFix message with a
  geometry_msgs/PoseStamped message. The latitude is stored in
  pose.position.y, the longitude is stored in pose.position.x, and the
  altitude is stored in pose.position.z. As before, the local xy frame is
  fixed in rotation such that the Z axis points away from the center of
  the Earth and the Y axis points north. However, the choice of
  geometry_msgs/PoseStamped allows for headings to be added in the future.
* Refactors initialize origin and fixes a bug.
* Contributors: Edward Venator

0.0.14 (2017-04-11)
-------------------
* Merge pull request `#435 <https://github.com/swri-robotics/marti_common/issues/435>`_ from swri-robotics/initialize-origin-license
  Fix whitespace and license in initialize_origin.py
* Fix whitespace and license in initialize_origin.py
  Replace "all rights reserved" with standard BSD 3-clause text and remove trailing whitespace in initialize_origin.py
* Fixes `#431 <https://github.com/swri-robotics/marti_common/issues/431>`_

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

0.0.9 (2016-03-04)
------------------

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
