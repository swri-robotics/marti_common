^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
