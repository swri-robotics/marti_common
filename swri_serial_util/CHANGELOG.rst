^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_serial_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.7.4 (2025-04-14)
------------------

3.7.3 (2024-09-18)
------------------

3.7.2 (2024-09-16)
------------------

3.7.1 (2024-09-06)
------------------

3.7.0 (2024-09-06)
------------------
* Cleaning up package maintainer (`#721 <https://github.com/swri-robotics/marti_common/issues/721>`_)
* Contributors: David Anthony

3.6.1 (2023-09-11)
------------------
* Fixing Buildfarm Issues (`#716 <https://github.com/swri-robotics/marti_common/issues/716>`_)
  * Cleaning up code to fix build farm errors
* Contributors: David Anthony

3.6.0 (2023-08-22)
------------------

3.5.4 (2023-08-14)
------------------

3.5.3 (2023-06-07)
------------------

3.5.2 (2023-05-30)
------------------

3.5.1 (2022-11-29)
------------------

3.5.0 (2022-10-11)
------------------

3.4.2 (2022-10-10)
------------------

3.3.2 (2020-11-24)
------------------
* Make SerialPort's functions virtual (`#608 <https://github.com/swri-robotics/marti_common/issues/608>`_)
* Contributors: Ryan DelGizzi

3.3.1 (2020-08-19)
------------------

3.3.0 (2020-07-15)
------------------

3.2.1 (2020-06-10)
------------------
* ROS Foxy support (`#582 <https://github.com/swri-robotics/marti_common/issues/582>`_)
* Contributors: P. J. Reed

3.2.0 (2020-05-13)
------------------

3.1.0 (2020-03-30)
------------------

3.0.5 (2020-03-10)
------------------

3.0.4 (2020-03-05)
------------------

3.0.3 (2019-11-11)
------------------

3.0.2 (2019-11-11)
------------------

3.0.1 (2019-11-11)
------------------

3.0.0 (2019-11-08)
------------------
* ROS2 Dashing conversion (`#549 <https://github.com/pjreed/marti_common/issues/549>`_)
* Contributors: P. J. Reed

2.10.0 (2019-09-04)
-------------------

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
