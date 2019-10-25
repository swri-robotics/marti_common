#!/usr/bin/env bash

apt-get update
rosdep update
apt-get install -y build-essential python3-colcon-ros

pushd /ws/src
git clone --depth 1 -b 2.0.0 https://github.com/ros/diagnostics.git
popd

cd /ws

rosdep install src --from-paths -i -y
colcon build
