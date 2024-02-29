#!/usr/bin/env python3
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

#detect only 16h5 tags with id= 0 to  30
cfg_16h5 = {
    "image_transport": "raw",
    #"image_transport": "compressed",
    "family": "16h5",
    "size": 0.0762,
    "z_up": True,
    "blur": 1.0,
    "tag_ids": range(30),
    "tag_frames": ["marker"]*30,

    "debug": 0 , 
    "max_hamming": 0,    
    "decimate": 1.0,
    "threads": 1,
    "refine-decode": 0 ,
    "refine-pose": 0 ,
    #"tag_size": [0.0762],
}

def generate_launch_description():
    composable_node = ComposableNode(
        node_name='apriltag',
        package='apriltag_ros', node_plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/camera/image_raw"), ("/apriltag/camera_info", "/camera/camera_info")],
        parameters=[cfg_16h5])

    container = ComposableNodeContainer(
        node_name='tag_container',
        node_namespace='apriltag',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
    