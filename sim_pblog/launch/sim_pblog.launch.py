# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


package_name = 'sim_pblog'


def generate_launch_description():
    loghome_launch_arg = DeclareLaunchArgument(
        'loghome', default_value=['~/.pblogs'],
        description='root log directory'
    )

    logdir_launch_arg = DeclareLaunchArgument(
        'logdir', default_value=[''],
        description='specific log directory in loghome'
    )

    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'sim_pblog.yaml'
    )

    node = Node(
        package=package_name,
        name='sim_pblog',
        executable='sim_pblog',
        arguments=[
            '--loghome', LaunchConfiguration('loghome'),
            '--logdir', LaunchConfiguration('logdir')
        ],
        parameters=[config]
    )

    ld = LaunchDescription()
    ld.add_action(loghome_launch_arg)
    ld.add_action(logdir_launch_arg)
    ld.add_action(node)

    return ld
