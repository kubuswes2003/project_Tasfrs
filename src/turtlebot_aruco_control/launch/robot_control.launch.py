#!/usr/bin/env python3
"""
Launch file for TurtleBot ArUco Control
Starts TurtleBot3 simulation, camera, and ArUco controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate launch description for TurtleBot ArUco control system
    """
    
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) or real robot'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch camera node (usb_cam)'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo world to load (empty_world, turtlebot3_world, etc.)'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Show debug window with ArUco detection'
    )
    
    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim')
    use_camera = LaunchConfiguration('use_camera')
    world = LaunchConfiguration('world')
    debug = LaunchConfiguration('debug')
    
    # Get package paths
    pkg_turtlebot_aruco = FindPackageShare('turtlebot_aruco_control')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_turtlebot_aruco,
        'config',
        'params.yaml'
    ])
    
    # TurtleBot3 Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        condition=IfCondition(use_sim)
    )
    
    # USB Camera node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera',
            'io_method': 'mmap',
            'framerate': 30.0,
        }],
        condition=IfCondition(use_camera)
    )
    
    # ArUco Controller node
    aruco_controller = Node(
        package='turtlebot_aruco_control',
        executable='aruco_controller',
        name='aruco_controller',
        output='screen',
        parameters=[config_file, {'debug': debug}],
        remappings=[
            ('/camera/image_raw', '/image_raw'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_arg,
        use_camera_arg,
        world_arg,
        debug_arg,
        
        # Nodes
        gazebo_launch,
        camera_node,
        aruco_controller,
    ])
