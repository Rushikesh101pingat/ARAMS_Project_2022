#!/usr/bin/env python3

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect only 16h5 tags with id=0
cfg_16h5 = {
    "image_transport": "raw",
    "family": "16h5",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
    #"tag_ids": [0],
    #"tag_frames": ["marker"],
    "tag_size": [0.162],
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/camera/image_raw"), ("/apriltag/camera_info", "/camera/camera_info")],
        parameters=[cfg_16h5])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])