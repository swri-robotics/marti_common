^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.0.0 (2017-08-02)
------------------
* Add support for boost::function callbacks to swri::Subscriber.
* Contributors: Elliot Johnson

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Contributors: P. J. Reed

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
* Fixes some compiler warnings.
* The swri::Subscriber::blockTimeouts function now returns a value.
  Previously, it did not explicitly returning a value, which has
  undefined behavior. It now returns the result of the blockTimeouts
  function that it wraps.
* Contributors: Edward Venator, Elliot Johnson, P. J. Reed

0.0.14 (2017-04-11)
-------------------
* Increase queue_size in swri_roscpp/Subscriber.
  This commit increases the queue size for subscribers that use the
  store mechanism instead of a callback.  The queue size was set to 1,
  which we have seen problems with, so this will increase it to 2.
* Add missing qualifiers for swri_roscpp unused parameter functions.
* Merge pull request `#385 <https://github.com/swri-robotics/marti_common/issues/385>`_ from evenator/unused-parameter-warnings
  Adds the ability to warn when unused parameters are set in a namespace.
* Add test for getUnusedParamKeys
  Adds an automated test for getUnusedParamKeys based on the example
  code.
* Remove default value of node handle for warnUnusedParams
  This default value may cause unexpected behvavior, especially
  with nodelets.
* Rename param_test to param_example.
  param_test isn't an automated test, just an example of how to use the
  param utilities.
* Document unused parameter functions.
  The set difference algorithms used to determine which parameters
  are used are non-obvious. This adds documentation.
* Mark _used_params static.
* Adds the ability to warn when unused parameters are set in a namespace.
  A common error when using unfamiliar ROS nodes is to accidentally set
  parameters by the wrong name. This features allows the node author
  to output a WARNING for any unused parameters.
  See the param_test node for an example.
* Contributors: Ed Venator, Edward Venator, Elliot Johnson, elliotjo

0.0.13 (2016-10-23)
-------------------
* Add swri_roscpp functions for reading float values.
  These add support for reading float values directly instead of
  doubles.
* Contributors: Elliot Johnson

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------
* Deprecate LatchedSubscriber.
  This commit adds an alternative to LatchedSubscriber and deprecates
  the LatchedSubscriber interface.  LatchedSubscriber should be replaced
  with a swri::Subscriber that is initialized with the address of a
  location to store messages.  For example, instead of:
  swri::LatchedSubscriber<my_package::MyMessage> msg\_;
  ...
  msg\_.initialize(nh\_, "topic_name");
  ...
  ROS_INFO("msg->field = %f", msg->field);
  this becomes:
  swri::Subscriber sub\_;
  my_package::MyMessageConstPtr msg\_;
  ...
  sub\_ = swri::SubscribeR(nh\_, "topic_name", &msg\_);
  ...
  ROS_INFO("msg->field = %f", msg->field).
  This change makes for a simpler and more consistent interface, and
  avoids the confusion that comes from overloading the -> operator.

0.0.9 (2016-03-04)
------------------

0.0.8 (2016-01-06)
------------------
* Makes swri::Subscriber's assignment operator return a value.
  According to the C++ spec, assignment operators must return a reference to the
  current object (\*this). swri::Subscriber's assignment operator was not
  returning a value, which works in GCC, but not Clang. GCC would do the right
  thing for you, but Clang will compile the code but generate a SIGILL 
  exception at runtime. This is easily fixed by manually returning \*this.
* Contributors: P. J. Reed

0.0.7 (2015-11-18)
------------------
* Fixes broken changelog `#279 <https://github.com/swri-robotics/marti_common/issues/279>`_.

0.0.6 (2015-11-17)
------------------
* First release of swri_roscpp
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
