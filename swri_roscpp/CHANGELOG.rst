^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4 (2017-04-11)
------------------
* Port `#385 <https://github.com/swri-robotics/marti_common/issues/385>`_ and `#419 <https://github.com/swri-robotics/marti_common/issues/419>`_ to kinetic. (`#420 <https://github.com/swri-robotics/marti_common/issues/420>`_)
  A common error when using unfamiliar ROS nodes is to accidentally set parameters
  by the wrong name. This feature allows the node author to output a WARNING
  for any unused parameters.
  Ported forward from indigo-devel
* Contributors: Edward Venator

0.2.3 (2016-12-09)
------------------

0.2.2 (2016-12-07)
------------------
* Deprecate LatchedSubscriber. (`#391 <https://github.com/swri-robotics/marti_common/issues/391>`_)
  LatchedSubscriber should be replaced with a swri::Subscriber that is
  initialized with the address of a location to store messages. This change
  makes for a simpler and more consistent interface, and avoids the confusion
  that comes from overloading the -> operator.
* Contributors: P. J. Reed

0.2.1 (2016-10-23)
------------------

0.2.0 (2016-06-21)
------------------

0.1.5 (2016-05-13)
------------------

0.1.4 (2016-05-12)
------------------
* Add timeoutParam() method to swri::Subscriber.
  This commit adds a new convenience method, timeoutParam, to
  swri::Subscriber that reads a specified parameter directly from the
  parameter server and sets it as the subscriber's timeout value.  This
  is to simplify setup code that currently has to define a temporary
  variable, read the parameter in the temp, and then set the timeout.
* Contributors: Elliot Johnson

0.1.3 (2016-03-04)
------------------
* Adds getParam() functions to swri_roscpp.
  These functions wrap NodeHandle::getParam(). If the parameter does
  not exist, they emit an error message and return false.
* Contributors: Edward Venator

0.1.2 (2016-01-06)
------------------

0.1.1 (2015-11-17)
------------------
* First jade release of swri_roscpp
* Contributors: Edward Venator

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
