# Copyright 2023 AIT - Austrian Institute of Technology GmbH
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
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the postgis_ros_bridge_publisher_node."""
    pkg_share_dir = get_package_share_directory('postgis_ros_bridge_demo')
    config_file = os.path.join(pkg_share_dir, 'gravel_quarry.yaml')

    pub_string_lat_lon = '"{' + 'latitude: 48.2345708, longitude: 14.4261733, altitude: 0.0, '
    pub_string_header = 'header: {stamp: now, frame_id: gps_sensor}' + '}"'

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='start rviz'),
        Node(
            package='postgis_ros_bridge',
            executable='postgis_ros_bridge_publisher_node',
            name='postgis_ros_publisher',
            parameters=[config_file],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', 'gps_sensor'],
        ),

        ExecuteProcess(
            cmd=[['ros2 ',
                  'topic ',
                  'pub ',
                  '/gps/satellite_plugin ',
                  'sensor_msgs/msg/NavSatFix ',
                  pub_string_lat_lon + pub_string_header,
                  ]],
            shell=True,
        ),
        Node(
            package='postgis_ros_bridge_demo',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{'lon': 14.4261733}, {'lat': 48.2345708}],
            remappings=[('/gps/fix', '/novatel_gps_1/fix')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=['-d', os.path.join(pkg_share_dir, 'gravel_quarry.rviz')],
        ),
    ])
