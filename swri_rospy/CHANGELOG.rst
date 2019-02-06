^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_rospy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.0.14 (2017-04-11)
-------------------

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------

0.0.10 (2016-05-12)
-------------------
* Adds swri_rospy subclasses for Subscriber, Service, and Timer
  These classes are drop-in replacements for their rospy counterparts,
  except that their constructor has an additional parameter
  asynchronous, which defaults to False. If asynchronous is false, the
  callback is wrapped with the single_threaded decorator. (This means
  that all of these classes use the single-thread callback queue by
  default.)
  Also, swri_rospy.spin is now injected into the rospy namespace to
  override rospy.spin. This is done to prevent users from accidentally
  using rospy.spin, which does not process the callback queue.
* Adds deadlock protection to single_threaded decorator.
  Decorating a function with single_threaded more than once would cause the
  callback queue to deadlock. This prevents recursive decoration.
* Adds exception handling to single_threaded decorator.
* Makes single_threaded decorator work for arbitrary ags
* Adds service_wrapper decorator.
  Also fixes line endings.
* Updates swri_rospy single_threaded example.
  Also fixes line-endings.
* Creates a new package swri_rospy.
  swri_rospy adds a new callback-based subscription, timer, and service
  capability to rospy.
* Contributors: Ed Venator, Edward Venator

* Adds swri_rospy subclasses for Subscriber, Service, and Timer
  These classes are drop-in replacements for their rospy counterparts,
  except that their constructor has an additional parameter
  asynchronous, which defaults to False. If asynchronous is false, the
  callback is wrapped with the single_threaded decorator. (This means
  that all of these classes use the single-thread callback queue by
  default.)
  Also, swri_rospy.spin is now injected into the rospy namespace to
  override rospy.spin. This is done to prevent users from accidentally
  using rospy.spin, which does not process the callback queue.
* Adds deadlock protection to single_threaded decorator.
  Decorating a function with single_threaded more than once would cause the
  callback queue to deadlock. This prevents recursive decoration.
* Adds exception handling to single_threaded decorator.
* Makes single_threaded decorator work for arbitrary ags
* Adds service_wrapper decorator.
  Also fixes line endings.
* Updates swri_rospy single_threaded example.
  Also fixes line-endings.
* Creates a new package swri_rospy.
  swri_rospy adds a new callback-based subscription, timer, and service
  capability to rospy.
* Contributors: Ed Venator

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
