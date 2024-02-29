#!/usr/bin/env python3

import os,time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    my_robot_slam = get_package_share_directory('my_robot_slam')
    #april_tag_ros = get_package_share_directory('april_tag_ros')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_robot_slam, 'localization.launch.py')
            ),
        ),
        Node(
            package='my_robot_slam',
            executable='set_initial_pose',
            name='set_initial_pose',
            output='screen'),
        #time.sleep(5),
        # Node(
        #     package='tf_listener',
        #     executable='follower',
        #     name='follower',
        #     output='screen'),
        # Node(
        #     package='my_robot_slam',
        #     executable='auto_explorer',
        #     name='auto_explorer',
        #     output='screen'),
    ])
