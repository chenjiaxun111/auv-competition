"""Actuation-isolated competition controller driven by the real ZED topic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('auv_competition_control')
    params = os.path.join(share, 'config', 'competition_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('target_ball_color', default_value='red'),
        DeclareLaunchArgument('red_ball_class', default_value='red_ball'),
        DeclareLaunchArgument('blue_ball_class', default_value='blue_ball'),
        DeclareLaunchArgument('gate_class', default_value='gate'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('assume_initial_depth', default_value='true'),
        DeclareLaunchArgument('objects_topic', default_value='/zed/objects3d'),
        DeclareLaunchArgument('log_rate_hz', default_value='2.0'),
        Node(
            package='auv_competition_control',
            executable='competition_controller',
            name='competition_shadow_controller',
            output='screen',
            parameters=[
                params,
                {
                    'target_ball_color': LaunchConfiguration('target_ball_color'),
                    'red_ball_class': LaunchConfiguration('red_ball_class'),
                    'blue_ball_class': LaunchConfiguration('blue_ball_class'),
                    'gate_class': LaunchConfiguration('gate_class'),
                    'objects_topic': LaunchConfiguration('objects_topic'),
                    'shadow_mode': True,
                    'shadow_auto_start': ParameterValue(
                        LaunchConfiguration('auto_start'), value_type=bool),
                    'shadow_assume_initial_depth': ParameterValue(
                        LaunchConfiguration('assume_initial_depth'),
                        value_type=bool),
                    'shadow_log_rate_hz': ParameterValue(
                        LaunchConfiguration('log_rate_hz'), value_type=float),
                },
            ],
        ),
    ])
