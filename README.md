marti_common
==============

[![Build Status](https://travis-ci.org/swri-robotics/marti_common.svg?branch=kinetic-devel)](https://travis-ci.org/swri-robotics/marti_common)

This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).

## Installation (ROS Indigo, Jade)

If you have installed ROS Indigo or Jade, you can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-$ROS_DISTRO-<package>

Building From Source (ROS Indigo, Jade, Kinetic)
------------

These directions assume you have already set up a catkin workspace. See [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the ROS Wiki for help setting up a catkin workspace.

### Checking out the source code (wstool)

If you're using wstool, add this repository to your wstool workspace:

    # Replace $ROS_DISTRO with indigo, jade, or kinetic as appropriate
    wstool set marti_common --git https://github.com/swri-robotics/marti_common.git -v $ROS_DISTRO-devel

### Checking out the source code (git)

If you're not using wstool, you can check out the repositories with git:

    # Replace $ROS_DISTRO with indigo, jade, or kinetic as appropriate
    git clone https://github.com/swri-robotics/marti_common.git --branch $ROS_DISTRO-devel

### Installing dependencies and building

Install all of the dependencies using rosdep by running the following command from the root of your catkin workspace:

    rosdep install --from-paths src --ignore-src

Build the workspace with catkin_make:

    catkin_make
