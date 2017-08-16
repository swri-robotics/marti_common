^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_geometry_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2017-08-02)
------------------

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Add OpenCV dependency
* Contributors: P. J. Reed

0.2.4 (2017-04-11)
------------------

0.2.3 (2016-12-09)
------------------
* Fix OpenCV dependencies for Kinetic build (`#400 <https://github.com/swri-robotics/marti_common/issues/400>`_)
* Contributors: P. J. Reed

0.2.2 (2016-12-07)
------------------

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------
* Update FindGEOS to generate linker flags correctly in Ubuntu 16.04 (`#348 <https://github.com/swri-robotics/marti_common/issues/348>`_).
  The regexes to find the link directory and library name from the
  output of geos-config were too liberal, so the library name
  regex would match on the `-linux` portion of the link directory,
  resulting in broken linker flags. This tightens up those regexes
  a bit to yield the correct library directory and name.
* Fix a typedef conflict in Ubuntu 16.04 (`#347 <https://github.com/swri-robotics/marti_common/issues/347>`_)
  Wrapping geos includes in #define statements forces geos to typedef
  int64 to int64_t so that it matches opencv's typedef.
* Add cubic spline interface for tf::Vector3 type.
* Contributors: Ed Venator, Marc Alban

0.1.5 (2016-05-13)
------------------
* Add an explicit dependency on pkg-config
* Contributors: P. J. Reed

0.1.4 (2016-05-12)
------------------
* Fixing Jade compilation issues
* Fixed compile error when ros-indigo-opencv3 is installed (`#307 <https://github.com/evenator/marti_common/issues/307>`_)
* Contributors: Kim Mathiassen
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

0.0.14 (2017-04-11)
-------------------

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------
* Adds cubic spline interface for tf::Vector3 type.
* Contributors: Marc Alban

0.0.11 (2016-05-13)
-------------------
* Adds explicit dependency on pkg-config
* Contributors: P. J. Reed

0.0.10 (2016-05-12)
-------------------

0.0.9 (2016-03-04)
------------------

0.0.8 (2016-01-06)
------------------

0.0.7 (2015-11-18)
------------------

0.0.6 (2015-11-17)
------------------
  Conflicts:
  swri_geometry_util/CMakeLists.txt
* Contributors: Kim Mathiassen, P. J. Reed

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
