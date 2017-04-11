^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2017-04-11)
------------------
* Increase queue_size in swri_roscpp/Subscriber.
  This commit increases the queue size for subscribers that use the
  store mechanism instead of a callback.  The queue size was set to 1,
  which we have seen problems with, so this will increase it to 2.
* Deprecate LatchedSubscriber. (`#392 <https://github.com/swri-robotics/marti_common/issues/392>`_)
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
* Contributors: Elliot Johnson, P. J. Reed

0.1.6 (2016-10-23)
------------------
* Add swri_roscpp functions for reading float values.
  These add support for reading float values directly instead of
  doubles.
* Contributors: Elliot Johnson

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
