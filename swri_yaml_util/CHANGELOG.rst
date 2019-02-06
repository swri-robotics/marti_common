^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_yaml_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

2.4.0 (2018-10-09)
------------------

2.3.0 (2018-05-25)
------------------

2.2.1 (2018-05-11)
------------------

2.2.0 (2018-02-12)
------------------

2.1.0 (2018-01-26)
------------------

2.0.0 (2017-12-18)
------------------

1.2.0 (2017-10-13)
------------------

1.1.0 (2017-08-31)
------------------

1.0.0 (2017-08-02)
------------------

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Contributors: P. J. Reed

0.2.4 (2017-04-11)
------------------
* Make swri_yaml_util build out-of-source.
* Contributors: Marc Alban

0.2.3 (2016-12-09)
------------------

0.2.2 (2016-12-07)
------------------

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------

0.1.5 (2016-05-13)
------------------
* Add an explicit dependency on pkg-config
* Contributors: P. J. Reed

0.1.4 (2016-05-12)
------------------
* Add support to load YAML from string and dictionary.
* Fix linking with yaml-cpp.
* Contributors: Elliot Johnson, Marc Alban

0.1.3 (2016-03-04)
------------------
* Adds uint16 support to swri_yaml_util
* Contributors: P. J. Reed

0.1.2 (2016-01-06)
------------------

0.1.1 (2015-11-17)
------------------

0.1.0 (2015-09-29)
------------------

0.0.14 (2017-04-11)
-------------------
* Add develspace include directory to swri_yaml_util
  Otherwise, version.h is missing and the package fails to build
* Make swri_yaml_util build out-of-source
  Fixes `#411 <https://github.com/swri-robotics/marti_common/issues/411>`_ by generating version.h in the devel space include folder instead of the source space.
  Based heavily on http://answers.ros.org/question/123221/
* Contributors: Edward Venator

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

0.0.9 (2016-03-04)
------------------

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
