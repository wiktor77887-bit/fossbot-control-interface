"""
Start the FOSSBot control interface.

Usage, e.g.:
    ros2 launch fossbot_control_interface interface.launch.py
    ros2 launch fossbot_control_interface interface.launch.py start_camera:=true
    ros2 launch fossbot_control_interface interface.launch.py use_real_robot:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('fossbot_control_interface'), 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_real_robot = LaunchConfiguration('use_real_robot')
    start_camera = LaunchConfiguration('start_camera')
    video_device = LaunchConfiguration('video_device')
    start_dashboard = LaunchConfiguration('start_dashboard')
    start_click_teleop = LaunchConfiguration('start_click_teleop')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with node parameters'),
        DeclareLaunchArgument('use_real_robot', default_value='false',
                              description='Enable the motors of the physical FOSSBot on start'),
        DeclareLaunchArgument('start_camera', default_value='false',
                              description='Also start usb_cam (/user_cam/image_raw)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Camera device used when start_camera:=true'),
        DeclareLaunchArgument('start_dashboard', default_value='true',
                              description='Start the live dashboard window'),
        DeclareLaunchArgument('start_click_teleop', default_value='true',
                              description='Start the click teleoperation window'),

        # Optional camera driver (external package). Normally it runs in the
        # driver container; this switch is handy for running everything locally.
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='user_cam',
            condition=IfCondition(start_camera),
            parameters=[{
                'video_device': video_device,
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv2rgb',
                'camera_name': 'user_cam',
            }],
            output='screen',
        ),

        Node(
            package='fossbot_control_interface',
            executable='click_teleop',
            name='click_teleop',
            condition=IfCondition(start_click_teleop),
            parameters=[
                params_file,
                {'enable_motors_on_start': ParameterValue(use_real_robot, value_type=bool)},
            ],
            output='screen',
        ),

        Node(
            package='fossbot_control_interface',
            executable='dashboard',
            name='dashboard',
            condition=IfCondition(start_dashboard),
            parameters=[params_file],
            output='screen',
        ),
    ])
