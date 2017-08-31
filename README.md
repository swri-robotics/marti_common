marti\_common [![Build Status](https://travis-ci.org/swri-robotics/marti_common.svg?branch=master)](https://travis-ci.org/swri-robotics/marti_common)
=============


This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).

Installation (ROS Indigo, Jade, Kinetic, Lunar)
-------------

If you have installed ROS Indigo, Jade, Kinetic, or Lunar, you can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-${ROS_DISTRO}-<package>

Building From Source (ROS Indigo, Jade, Kinetic, Lunar)
------------

These directions assume you have already set up a catkin workspace and rosdep. See [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki for help setting up a catkin workspace and the [rosdep documentation](http://wiki.ros.org/rosdep) on the ROS wiki for help setting up rosdep.

1. Check out the source code

    a. If you use wstool:
    ```bash
    wstool set marti_common --git https://github.com/swri-robotics/marti_common.git
    wstool update marti_common
    ```

	b. Using plain git:
    ```bash
    git clone https://github.com/swri-robotics/marti_common.git
    ```
2. Install dependencies:

    ```bash
    # (In the root of this repository)
    rosdep install --from-paths . --ignore-src
    ```

3. Build

    a. If you use catkin tools:
    ```bash
    catkin build
    ```

    b. Using plain catkin:
    ```bash
    catkin_make
    ```
