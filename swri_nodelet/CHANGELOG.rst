^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_nodelet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Ed Venator, Edward Venator

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
