^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_yaml_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------
* Adding an explicit dependency on pkg-config
* Contributors: P. J. Reed

0.0.10 (2016-05-12)
-------------------
* Add support to load YAML from string and dictionary.
* Fix linking with yaml-cpp.
* Contributors: Elliot Johnson, Marc Alban

0.0.9 (2016-03-04)
------------------
* Adds uint16 support to swri_yaml_util
* Contributors: P. J. Reed

0.0.8 (2016-01-06)
------------------

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------

0.0.5 (2015-09-27)
------------------
* Adds missing package dependency and find_package() for boost in 
  swri_yaml_util. See issue `#240 <https://github.com/evenator/marti_common/issues/240>`_
* Cleans up all catkin_lint by adding a proper package description 
  and fixing some formatting in CMakeLists.txt.
* Contributors: Ed Venator

0.0.4 (2015-09-27)
------------------
* Adds boost include directories to yaml_util because yaml-cpp uses boost and doesn't export the include directory.
* Contributors: Ed Venator

0.0.3 (2015-09-26)
------------------
* Fixes missing yaml-cpp dependency in swri_yaml_util.
  Refs `#233 <https://github.com/swri-robotics/marti_common/issues/233>`_.
* Contributors: Ed Venator

0.0.2 (2015-09-25)
------------------
* Renames yaml_util to swri_yaml_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
