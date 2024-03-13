#!/usr/bin/env python
# *****************************************************************************
#
# Copyright (c) 2024, Southwest Research Institute® (SwRI®)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# *****************************************************************************

import os
import unittest
import pytest

import launch
import launch_ros
import launch_testing
import ament_index_python
from launch_ros.actions import Node

def get_tests(*, args=[]):
    test_path = os.path.join(
            ament_index_python.get_package_prefix("swri_transform_util"),
            "lib/swri_transform_util",
            "local_xy_util_test",
    )

    return launch_testing.actions.GTest(
            path=[test_path],
            name="local_xy_util_test",
            additional_env={"PYTHONBUFFERED": "1"},
            output="screen",
    )

@pytest.mark.launch_test
def generate_test_description():
    init_origin = Node(
        package="swri_transform_util",
        name="initialize_origin",
        executable="initialize_origin.py",
        parameters=[{
            "local_xy_frame": "/far_field",
            "local_xy_origin": "swri",
            "local_xy_origins": [29.45196669, -98.61370577, 233.719, 0.0],
        }]
    )

    return launch.LaunchDescription(
            [
                init_origin,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
    )

class LocalXyUtilTest(unittest.TestCase):
    def test_local_xy_util(self, launch_service, proc_info, proc_output):
        tests = get_tests();
        with launch_testing.tools.launch_process(
            launch_service, tests, proc_info, proc_output
        ):
            proc_info.assertWaitForStartup(process=tests, timeout=30)
            proc_info.assertWaitForShutdown(process=tests, timeout=600)
        launch_testing.asserts.assertExitCodes(proc_info, process=tests)
