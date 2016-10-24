^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add timeoutParam() method to swri::Subscriber.
  This commit adds a new convenience method, timeoutParam, to
  swri::Subscriber that reads a specified parameter directly from the
  parameter server and sets it as the subscriber's timeout value.  This
  is to simplify setup code that currently has to define a temporary
  variable, read the parameter in the temp, and then set the timeout.
* Contributors: Elliot Johnson

0.0.9 (2016-03-04)
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
