marti\_common
![CI](https://github.com/swri-robotics/marti_common/workflows/CI/badge.svg)
![CI](https://github.com/swri-robotics/marti_common/workflows/CI/badge.svg?branch=dashing-devel)
[![ROS2 Build Farm Build Status](http://build.ros2.org/buildStatus/icon?job=Ddev__marti_common__ubuntu_bionic_amd64)](http://build.ros2.org/job/Ddev__marti_common__ubuntu_bionic_amd64/)
=============

This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).  This branch adds support for ROS 2 Dashing and newer releases.  Most packages from ROS 1 have been ported, but a few have been removed due to being unnecessary or redundant, and some functionality is not implemented yet.

Overview
--------

What's changed in the ROS 2 port?

Removed packages:
1. `marti_data_structures`  
    Nothing used this and it only contained a linked list
2. `swri_nodelet`  
    Obsolete due to ROS 2's component mechanism
3. `swri_rospy`  
    Unnecessary in ROS 2
4. `swri_string_util`  
    Equivalent functionality is provided by boost
5. `swri_yaml_util`  
    This package only existed in order to bridge nodes between ROS Hydro and ROS Indigo; use `yaml-cpp` directly now

Package migration notes:
1. `swri_image_util`  
    `replace_colors_node` has not been ported yet due to extensive changes in how ROS parameters work
2. `swri_roscpp`
    1. Many parameter-related classes have been removed; they are unnecessary due to `roscpp::Node::delcare_parameter` providing equivalent functionality now
    2. Topic services have not been ported yet
3. `swri_transform_util`  
    `initialize_origin.py` does not publish a tf frame due to tf2 Python bindings not being fully functional in ROS 2 Dashing
4. Launch files
    Launch files have not yet been migrated to ROS 2

Also note that many features have not been tested yet.  Please open an issue if you try to use something and it doesn't work.

Installation (ROS Foxy, Galactic, Humble)
------------

If you have installed ROS 2, you can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-${ROS_DISTRO}-<package>

Building From Source (ROS Foxy, Galactic, Humble)
------------

These directions assume you have already set up rosdep. See the [rosdep documentation](http://wiki.ros.org/rosdep) on the ROS wiki for help setting up rosdep.

1. If you don't have a colcon workspace, create one:

    ```bash
    mkdir $HOME/workspace/src
    cd $HOME/workspace/src
    ```

2. Check out the source code

    ```bash
    cd $HOME/workspace/src
    git clone https://github.com/swri-robotics/marti_common.git
    ```

3. Install dependencies:

    ```bash
    # (In the root of this repository)
    rosdep install --from-paths . --ignore-src
    ```

4. Build

    ```bash
    cd $HOME/workspace
    colcon build
    ```
