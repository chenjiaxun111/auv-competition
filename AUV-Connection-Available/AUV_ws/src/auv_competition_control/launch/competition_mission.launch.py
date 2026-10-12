"""Launch the competition controller, optional mock source, and ROV bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('auv_competition_control')
    bridge_share = get_package_share_directory('rov_thruster_bridge')
    use_mock = LaunchConfiguration('use_mock')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='false',
            description='Use synthetic ZED and depth data'),
        DeclareLaunchArgument(
            'target_ball_color', default_value='red',
            description='Competition ball color: red or blue'),
        Node(
            package='auv_competition_control',
            executable='competition_controller',
            name='competition_controller',
            output='screen',
            parameters=[
                os.path.join(control_share, 'config', 'competition_params.yaml'),
                {'target_ball_color': LaunchConfiguration('target_ball_color')},
            ],
        ),
        Node(
            package='auv_competition_control',
            executable='mock_objects3d',
            name='mock_objects3d',
            output='screen',
            condition=IfCondition(use_mock),
            parameters=[{
                'target_ball_color': LaunchConfiguration('target_ball_color'),
            }],
        ),
        Node(
            package='rov_thruster_bridge',
            executable='thruster_bridge_node',
            namespace='auv',
            name='rov_bridge',
            output='screen',
            condition=UnlessCondition(use_mock),
            parameters=[os.path.join(
                bridge_share, 'config', 'thruster_params.yaml')],
        ),
    ])
