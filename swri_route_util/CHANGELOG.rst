^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_route_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.2 (2020-11-24)
------------------

3.3.1 (2020-08-19)
------------------

3.3.0 (2020-07-15)
------------------

3.2.1 (2020-06-10)
------------------
* ROS Foxy support (`#582 <https://github.com/swri-robotics/marti_common/issues/582>`_)
* Contributors: P. J. Reed

3.2.0 (2020-05-13)
------------------
* Add overload of generateObstacleData for tracked objects (Dashing) (`#579 <https://github.com/swri-robotics/marti_common/issues/579>`_)
* Contributors: Matthew Bries

3.1.0 (2020-03-30)
------------------

3.0.5 (2020-03-10)
------------------
* Fix linking bugs (`#569 <https://github.com/swri-robotics/marti_common/issues/569>`_)
* Contributors: P. J. Reed

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
* Remove incorrect translation of object geometry (`#527 <https://github.com/swri-robotics/marti_common/issues/527>`_)
* Contributors: agyoungs

2.5.0 (2018-10-12)
------------------

2.4.0 (2018-10-09)
------------------
* Update package maintainers (`#520 <https://github.com/swri-robotics/marti_common/issues/520>`_)
* Contributors: P. J. Reed

2.3.0 (2018-05-25)
------------------
* Fill in route id of route positions when possible. (`#517 <https://github.com/swri-robotics/marti_common/issues/517>`_)
* Contributors: Marc Alban

2.2.1 (2018-05-11)
------------------
* Catch and fix NaNs in the fillOrientations function (`#513 <https://github.com/swri-robotics/marti_common/issues/513>`_)
* Contributors: kriskozak

2.2.0 (2018-02-12)
------------------

2.1.0 (2018-01-26)
------------------

2.0.0 (2017-12-18)
------------------
* Accept '1' or 'true' for stop points. (`#489 <https://github.com/swri-robotics/marti_common/issues/489>`_)
* Contributors: Marc Alban

1.2.0 (2017-10-13)
------------------
* Add support for vehicle_width_override property on route (`#485 <https://github.com/swri-robotics/marti_common/issues/485>`_)
* Add bounds checking to extractSubroute. (`#486 <https://github.com/swri-robotics/marti_common/issues/486>`_)
* Contributors: Marc Alban, Matthew Bries

1.1.0 (2017-08-31)
------------------

1.0.0 (2017-08-02)
------------------
* Add route speed functions (`#466 <https://github.com/swri-robotics/marti_common/issues/466>`_)
  * Add visualization function for swri_route_util.
  * Add code to calculate max speeds based on curvature to swri_route_util.
  * Add speed/obstacle functionality to swri_route_util.
* Add extractSubroute function.
* Contributors: Elliot Johnson, elliotjo

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/swri-robotics/marti_common/issues/443>`_)
* Contributors: P. J. Reed

0.2.4 (2017-04-11)
------------------

0.2.3 (2016-12-09)
------------------

0.2.2 (2016-12-07)
------------------
* Add support for stop point metadata.
* Add helper method to find files within a directory and subdirectories based on regular expression matching for the filename.
* Add sru::projectOntoRouteWindow (`#393 <https://github.com/swri-robotics/marti_common/issues/393>`_)
  This is a utility function to project a point onto a window of the
  route.
* Contributors: P. J. Reed

0.2.1 (2016-10-23)
------------------
* Changing the order of include dirs
  "${catkin_INCLUDE_DIRS}" needs to be listed after "include", otherwise gcc may
  try to compile this component's cpp files using headers from a system-installed
  version of swri_route_util.
* Contributors: P. J. Reed

0.2.0 (2016-06-21)
------------------
* Add error message for non-unique route point IDs.
* Contributors: Elliot Johnson, P. J. Reed

0.1.5 (2016-05-13)
------------------

0.1.4 (2016-05-12)
------------------
* Fix distances in routeDistances for points before start point.
  There were two bugs in routeDistances that were causing the incorrect
  distance to be calculated for points before the start point.  An error
  in the iteration bounds was causing the distance of the first point to
  be 0.0.  Secondly, the arc length for the other points was just the
  relative distance between two points instead of the cummulative
  distance.
* Merge pull request `#331 <https://github.com/evenator/marti_common/issues/331>`_ from elliotjo/sru-add-distance-functions-jade
  Add util functions to calculate distances between route points. (jade)
* Remove commented out code in swri_route_util.
* Add util functions to calculate distances between route points.
  This commit adds two utility functions to calculate the distances (in
  terms of arc length) between route points.  One function calculates
  the distance between two points, the other calculates the distance
  between one point and many other points and should provide much better
  performance for that common need.
* Add native-ish ROS serialization support to sru::Route.
  This commit adds native(-ish) ROS serialization support so that
  swri_route_util::Route can be used directly with publishers and
  subscribers. This is purely for convenience rather than performance
  (although you will get improved performance in nodelets that
  publish/subscribe by avoiding serialization).  Under the hood, the
  implementation does serialization with the native type and then
  converts it to/from the swri_route_util::Route type.
  This commit also fixes a missing special case in
  interpolateRouteSegment (0 < distance < 1) and reorganized the if/else
  blocks to be clearer.
* Add swri_route_util package.
  This commit adds a new package called swri_route_util that provides a
  more user-friendly interface to the marti_nav_msgs Route and RoutPoint
  classes, and a set of useful utilities.  At this point, most of the
  code (except the properties) has been well tested on bag files.
* Fix distances in routeDistances for points before start point.
  There were two bugs in routeDistances that were causing the incorrect
  distance to be calculated for points before the start point.  An error
  in the iteration bounds was causing the distance of the first point to
  be 0.0.  Secondly, the arc length for the other points was just the
  relative distance between two points instead of the cummulative
  distance.
* Merge pull request `#330 <https://github.com/evenator/marti_common/issues/330>`_ from elliotjo/sru-add-distance-functions-indigo
  Add util functions to calculate distances between route points. (indigo)
* Remove commented out code in swri_route_util.
* Add util functions to calculate distances between route points.
  This commit adds two utility functions to calculate the distances (in
  terms of arc length) between route points.  One function calculates
  the distance between two points, the other calculates the distance
  between one point and many other points and should provide much better
  performance for that common need.
* Add native-ish ROS serialization support to sru::Route.
  This commit adds native(-ish) ROS serialization support so that
  swri_route_util::Route can be used directly with publishers and
  subscribers. This is purely for convenience rather than performance
  (although you will get improved performance in nodelets that
  publish/subscribe by avoiding serialization).  Under the hood, the
  implementation does serialization with the native type and then
  converts it to/from the swri_route_util::Route type.
  This commit also fixes a missing special case in
  interpolateRouteSegment (0 < distance < 1) and reorganized the if/else
  blocks to be clearer.
* Add swri_route_util package.
  Adds a new package called swri_route_util that provides a
  more user-friendly interface to the marti_nav_msgs Route and RoutPoint
  classes, and a set of useful utilities.  At this point, most of the
  code (except the properties) has been well tested on bag files.
* Contributors: Elliot Johnson, Marc Alban

0.1.3 (2016-03-04)
------------------

0.1.2 (2016-01-06)
------------------

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
* Changes the order of include dirs
  "${catkin_INCLUDE_DIRS}" needs to be listed after "include", otherwise gcc may
  try to compile this component's cpp files using headers from a system-installed
  version of swri_route_util.
* Adds support for stop point metadata.
* Adds sru::projectOntoRouteWindow, a utility function to project a point onto a
  window of the route.
* Fixes projectOntoRoute to return a normalized route coordinate
  when the point is past the end of the route.
* Fixes a major bug in nearestDistanceToLineSegment that was
  affecting projectOntoRoute.  A misnamed variable v_len was actually
  the square of v_len and caused the reported distance along the route
  segment to be the square of the desired answer.  Chanes the code to take the
  appropriate square root and changes the variable name to avoid
  confusion in the future.
* Adds an error check when a sru::Route rebuilds its point

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------
* Contributors: Elliot Johnson

0.0.9 (2016-03-04)
------------------

0.0.8 (2016-01-06)
------------------

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------

0.0.5 (2015-09-27 15:27)
------------------------

0.0.4 (2015-09-27 11:35)
------------------------

0.0.3 (2015-09-26)
------------------

0.0.2 (2015-09-25 15:00)
------------------------

0.0.1 (2015-09-25 09:06)
------------------------
