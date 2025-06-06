# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch rviz2 for ROS2-OpenVINO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

from launch.substitutions import LaunchConfiguration, PythonExpression
import launch

def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('openvino_node'), 'launch',
                                'rviz/default.rviz')
    return LaunchDescription([
        # Rviz
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]),
    ])
