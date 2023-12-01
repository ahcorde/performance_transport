#!/usr/bin/env python3
# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing

import pytest


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    image_path = os.path.join(get_package_share_directory('performance_transport'),
                              'resources', 'image.bmp')
    print(image_path)

    publish_image_node = Node(
        package='performance_transport',
        executable='publish_image',
        parameters=[{'filename': LaunchConfiguration('filename'),
                     'size': LaunchConfiguration('size'),
                     'compress': LaunchConfiguration('compress'),
                     'compress_type': LaunchConfiguration('compress_type'),
                     'loop_time': LaunchConfiguration('loop_time'),
                     'transport_hint': LaunchConfiguration('transport_type'),
                     'rosbag_topic': LaunchConfiguration('rosbag_topic'),
                     'output_name': LaunchConfiguration('output_name'),
                     'camera.image.enable_pub_plugins': [['image_transport/',
                                                          LaunchConfiguration('transport_type')]]
                     }],
        output='screen')

    subscriber_image_node = Node(
        package='performance_transport',
        executable='subscribe_image',
        parameters=[{'compress_type': LaunchConfiguration('compress_type'),
                     'transport_hint': LaunchConfiguration('transport_type'),
                     'loop_time': LaunchConfiguration('loop_time'),
                     'output_name': LaunchConfiguration('output_name')}],
        output='screen')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'compress',
            default_value=['10'],
            description='Compress value',
        ),
        DeclareLaunchArgument(
            'rosbag_topic',
            default_value=[''],
            description='Rosbag topic',
        ),
        DeclareLaunchArgument(
            'output_name',
            default_value=[''],
            description='Output name',
        ),
        DeclareLaunchArgument(
            'compress_type',
            default_value=[''],
            description='Compress type',
        ),
        DeclareLaunchArgument(
            'filename',
            default_value=[''],
            description='Video or rosbag file',
        ),
        DeclareLaunchArgument(
            'size',
            default_value=['4096'],
            description='Image size value',
        ),
        DeclareLaunchArgument(
            'transport_type',
            default_value=[''],
            description='Transport type',
        ),
        DeclareLaunchArgument(
            'loop_time',
            default_value=['300'],
            description='Loop time',
        ),
        subscriber_image_node,
        publish_image_node,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class TestFixture(unittest.TestCase):

    def test_arm(self, proc_info, proc_output, publish_image_node, subscriber_image_node):
        proc_info.assertWaitForShutdown(process=publish_image_node, timeout=(30 * 11))
        proc_info.assertWaitForShutdown(process=subscriber_image_node, timeout=(30 * 11))


# These tests are run after the processes in generate_test_description() have shutdown.
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, publish_image_node):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info, process=publish_image_node,
                                               allowable_exit_codes=[0])
