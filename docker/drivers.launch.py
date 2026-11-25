"""
Start the drivers: the robot (simulation or bridge) and the laptop camera.

Runs in the driver container (docker/Dockerfile.drivers), e.g.:
    ros2 launch /drivers/drivers.launch.py robot:=sim world:=simple_shapes.sdf
    ros2 launch /drivers/drivers.launch.py robot:=bridge
    ros2 launch /drivers/drivers.launch.py robot:=none      # camera only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='none',
                              choices=['sim', 'bridge', 'none'],
                              description='Robot driver: Gazebo simulation, '
                                          'fossbot-ros2-bridge or none (camera only)'),
        DeclareLaunchArgument('world', default_value='simple_shapes.sdf',
                              description='Gazebo world (robot:=sim)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz with the simulator (robot:=sim)'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start usb_cam (/user_cam/image_raw)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Laptop camera device'),

        # The package paths are only resolved for the selected driver, so the
        # simulator image does not need the bridge and vice versa.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fossbot_educational_description'),
                'launch', 'single.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
            condition=LaunchConfigurationEquals('robot', 'sim'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fossbot_bridge'), 'launch', 'bringup.launch.py'])),
            condition=LaunchConfigurationEquals('robot', 'bridge'),
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='user_cam',
            condition=IfCondition(LaunchConfiguration('start_camera')),
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv2rgb',
                'camera_name': 'user_cam',
            }],
            output='screen',
        ),
    ])
