^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_transform_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
