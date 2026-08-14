MARTI Common
--------

This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).  This branch adds support for ROS 2 Dashing and newer releases.  Most packages from ROS 1 have been ported, but a few have been removed due to being unnecessary or redundant, and some functionality is not implemented yet.

Build Status
--------
ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Humble** | [`humble`](https://github.com/swri-robotics/marti_common/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/marti_common/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/marti_common/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Hdev__marti_common__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__marti_common__ubuntu_jammy_amd64/) | [swri_cli_tools](https://index.ros.org/p/swri_cli_tools/#humble) <br /> [swri_console_util](https://index.ros.org/p/swri_console_util/#humble) <br /> [swri_dbw_interface](https://index.ros.org/p/swri_dbw_interface/#humble) <br /> [swri_geometry_util](https://index.ros.org/p/swri_geometry_util/#humble) <br /> [swri_image_util](https://index.ros.org/p/swri_image_util/#humble) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#humble) <br /> [swri_opencv_util](https://index.ros.org/p/swri_opencv_util/#humble) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#humble) <br /> [swri_roscpp](https://index.ros.org/p/swri_roscpp/#humble) <br /> [swri_route_util](https://index.ros.org/p/swri_route_util/#humble) <br /> [swri_serial_util](https://index.ros.org/p/swri_serial_util/#humble) <br /> [swri_transform_util](https://index.ros.org/p/swri_transform_util/#humble)
**Jazzy** | [`jazzy`](https://github.com/swri-robotics/marti_common/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/marti_common/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/marti_common/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Jdev__marti_common__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__marti_common__ubuntu_noble_amd64/) | [swri_cli_tools](https://index.ros.org/p/swri_cli_tools/#jazzy) <br /> [swri_console_util](https://index.ros.org/p/swri_console_util/#jazzy) <br /> [swri_dbw_interface](https://index.ros.org/p/swri_dbw_interface/#jazzy) <br /> [swri_geometry_util](https://index.ros.org/p/swri_geometry_util/#jazzy) <br /> [swri_image_util](https://index.ros.org/p/swri_image_util/#jazzy) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#jazzy) <br /> [swri_opencv_util](https://index.ros.org/p/swri_opencv_util/#jazzy) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#jazzy) <br /> [swri_roscpp](https://index.ros.org/p/#jazzy) <br /> [swri_route_util](https://index.ros.org/p/swri_route_util/#jazzy) <br /> [swri_serial_util](https://index.ros.org/p/swri_serial_util/#jazzy) <br /> [swri_transform_util](https://index.ros.org/p/swri_transform_util/#jazzy)
**Kilted** | [`kilted`](https://github.com/swri-robotics/marti_common/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/marti_common/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/marti_common/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Kdev__marti_common__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__marti_common__ubuntu_noble_amd64/) | [swri_cli_tools](https://index.ros.org/p/swri_cli_tools/#kilted) <br /> [swri_console_util](https://index.ros.org/p/kiltedswri_console_util/#kilted) <br /> [swri_dbw_interface](https://index.ros.org/p/swri_dbw_interface/#kilted) <br /> [swri_geometry_util](https://index.ros.org/p/swri_geometry_util/#kilted) <br /> [swri_image_util](https://index.ros.org/p/swri_image_util/#kilted) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#kilted) <br /> [swri_opencv_util](https://index.ros.org/p/swri_opencv_util/#kilted) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#kilted) <br /> [swri_roscpp](https://index.ros.org/p/swri_roscpp/#kilted) <br /> [swri_route_util](https://index.ros.org/p/swri_route_util/#kilted) <br /> [swri_serial_util](https://index.ros.org/p/swri_serial_util/#kilted) <br /> [swri_transform_util](https://index.ros.org/p/swri_transform_util/#kilted)
**Lyrical** | [`lyrical`](https://github.com/swri-robotics/marti_common/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/marti_common/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/marti_common/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Ldev__marti_common__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__marti_common__ubuntu_resolute_amd64/) | [swri_cli_tools](https://index.ros.org/p/swri_cli_tools/#lyrical) <br /> [swri_console_util](https://index.ros.org/p/swri_console_util/#lyrical) <br /> [swri_dbw_interface](https://index.ros.org/p/swri_dbw_interface/#lyrical) <br /> [swri_geometry_util](https://index.ros.org/p/swri_geometry_util/#lyrical) <br /> [swri_image_util](https://index.ros.org/p/swri_image_util/#lyrical) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#lyrical) <br /> [swri_opencv_util](https://index.ros.org/p/swri_opencv_util/#lyrical) <br /> [swri_math_util](https://index.ros.org/p/swri_math_util/#lyrical) <br /> [swri_roscpp](https://index.ros.org/p/swri_roscpp/#lyrical) <br /> [swri_route_util](https://index.ros.org/p/swri_route_util/#lyrical) <br /> [swri_serial_util](https://index.ros.org/p/swri_serial_util/lyrical) <br /> [swri_transform_util](https://index.ros.org/p/swri_transform_util/#lyrical)

Installation (ROS 2 Humble, Jazzy, Kilted, Lyrical)
------------

If you have installed ROS 2, you can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-${ROS_DISTRO}-<package>

Building From Source (ROS 2 Humble, Jazzy, Kilted, Lyrical)
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
