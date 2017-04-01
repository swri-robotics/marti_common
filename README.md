marti_common
==============

[![Build Status](https://travis-ci.org/swri-robotics/marti_common.svg?branch=indigo-devel)](https://travis-ci.org/swri-robotics/marti_common)

This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).

Installation (ROS Indigo, Jade, Kinetic)
-------------

If you have installed ROS Indigo, Jade, or Kinetic, you can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-<distro>-<package>

Building From Source (ROS Indigo, Jade, Kinetic)
------------

These directions assume you have already set up a catkin workspace and rosdep. See [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki for help setting up a catkin workspace and the [rosdep documentation](http://wiki.ros.org/rosdep) on the ROS wiki for help setting up rosdep.

1. Check out the source code

    a. If you use wstool:
    ```
    # Replace $ROS_DISTRO with indigo, jade, or kinetic as appropriate
    wstool set marti_common --git https://github.com/swri-robotics/marti_common.git -v $ROS_DISTRO-devel
    wstool update marti_common
    ```

	b. Using plain git:
    ```
    # Replace $ROS_DISTRO with indigo, jade, or kinetic as appropriate
    git clone https://github.com/swri-robotics/marti_common.git --branch $ROS_DISTRO-devel
    ```
2. Install dependencies:
    ```
    # (In the root of this repository)
    rosdep install --from-paths . --ignore-src
    ```

3. Build

    a. If you use catkin tools:
    
    ```
    catkin build
    ```
    b. Using plain catkin:
    
    ```
    catkin_make
    ```
