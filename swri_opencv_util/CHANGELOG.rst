^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_opencv_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Mark some constructors explicit.
* Refactor RANSAC matching code to use more matrix operations.
* Fix bugs in FitRigidTransform2d.
  The main problem was that reshape was being incorrectly, causing the
  points to get shuffled around.  Once that was fixed, it was clear that
  the rotation should not be inverted.  Also added a comment to clarify
  the significance of the returned transform.
* Contributors: Elliot Johnson, Marc Alban

0.1.1 (2015-11-17)
------------------

0.1.0 (2015-09-29)
------------------

0.0.5 (2015-09-27)
------------------

0.0.4 (2015-09-27)
------------------

0.0.3 (2015-09-26)
------------------
* Clean up dependencies
  Remove unneeded ones, add required ones not specified
* Contributors: Jerry Towler

0.0.2 (2015-09-25)
------------------
* Renames opencv_util package to swri_opencv_util. Refs `#231 <https://github.com/swri-robotics/marti_common/issues/231>`_
* Contributors: Edward Venator

0.0.1 (2015-09-25)
------------------
