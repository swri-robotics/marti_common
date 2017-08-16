^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_serial_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2017-08-02)
------------------

0.3.0 (2017-06-20)
------------------
* Merge together the indigo, jade, and kinetic branches (`#443 <https://github.com/pjreed/marti_common/issues/443>`_)
* Contributors: P. J. Reed

0.2.4 (2017-04-11)
------------------
* Support higher serial baud rates
  The system header `/usr/include/asm-generic/termbits.h` has constants for
  supporting baud rates up to 4000000, but swri_serial_util only allows up
  to 230400.  Some devices support these higher rates and there's no reason
  to not support them, so this adds support for them.
* Check for error from ioctl in serial_port
  Fixes `#406 <https://github.com/swri-robotics/marti_common/issues/406>`_
* Contributors: Edward Venator, P. J. Reed

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

0.1.4 (2016-05-12)
------------------

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
* Support higher serial baud rates (`#423 <https://github.com/swri-robotics/marti_common/issues/423>`_)

0.0.13 (2016-10-23)
-------------------

0.0.12 (2016-08-14)
-------------------

0.0.11 (2016-05-13)
-------------------

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

0.0.4 (2015-09-27)
------------------

0.0.3 (2015-09-26)
------------------
* Fixes missing boost dependency in swri_serial_util.
  Refs `#234 <https://github.com/swri-robotics/marti_common/issues/234>`_.
* Contributors: Ed Venator

0.0.2 (2015-09-25)
------------------
* Renames serial_util to swri_serial_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_.
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
