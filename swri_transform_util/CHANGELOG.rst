^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_transform_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
