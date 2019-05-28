^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_nodelet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

2.4.0 (2018-10-09)
------------------
* Update package maintainers (`#520 <https://github.com/swri-robotics/marti_common/issues/520>`_)
* Contributors: P. J. Reed

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

0.2.3 (2016-12-09)
------------------

0.2.2 (2016-12-07)
------------------

0.2.1 (2016-10-23)
------------------
* Add C++ and CMake macros for wrapper nodes
  Defines a C++ macro to replace the normal nodelet export wrapper that
  also creates a factory function returning a pointer to that nodelet.
  Defines a CMake macro
  swri_nodelet_add_node(NODELET_NODENAME NODELET_NAMESPACE NODELET_CLASS)
  that automatically generates the c++ code for a node wrapper with node
  name NODELET_NODENAME that wraps the nodelet and makes a CMake target
  to build the node.
* Add tests for swri_nodelet with manager and standalone
* Contributors: Ed Venator

0.2.0 (2016-06-21)
------------------

0.1.5 (2016-05-13)
------------------

0.1.4 (2016-05-12)
------------------
* Add swri_nodelet package.
  This package simplifies launch files that can easily change between
  running nodelets in a shared manager or as standalone processes.
* Contributors: Elliot Johnson

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
* Include boost/make_shared.hpp
  This header is necessary in the unlikely event that the using cpp file hasn't included boost/make_shared.hpp anywhere else. Also, it's good practice to include what you use.
* Fix include_directories order in swri_roscpp
  Include package-local header files before catkin header files
  so that source is always built against source header files, even
  if the same package exists elsewhere in the catkin tree.
* Contributors: Edward Venator

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------
* Update version numbers for new packages.
* Version 0.0.10 changelogs.
* Add swri_nodelet package.
  This package simplifies launch files that can easily change between
  running nodelets in a shared manager or as standalone processes.
* Contributors: Edward Venator, Elliot Johnson

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
