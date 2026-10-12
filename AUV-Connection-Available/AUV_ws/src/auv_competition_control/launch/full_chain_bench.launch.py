"""Real ZED -> bounded planner -> bridge -> NanoPi bench integration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('auv_competition_control')
    bridge_share = get_package_share_directory('rov_thruster_bridge')
    return LaunchDescription([
        DeclareLaunchArgument('target_ball_color', default_value='red'),
        DeclareLaunchArgument('red_ball_class', default_value='red_ball'),
        DeclareLaunchArgument('blue_ball_class', default_value='blue_ball'),
        DeclareLaunchArgument('gate_class', default_value='gate'),
        Node(
            package='auv_competition_control',
            executable='competition_controller',
            name='competition_bench_controller',
            output='screen',
            parameters=[
                os.path.join(
                    control_share, 'config', 'competition_params.yaml'),
                {
                    'target_ball_color': LaunchConfiguration(
                        'target_ball_color'),
                    'red_ball_class': LaunchConfiguration('red_ball_class'),
                    'blue_ball_class': LaunchConfiguration('blue_ball_class'),
                    'gate_class': LaunchConfiguration('gate_class'),
                    'shadow_mode': True,
                    'shadow_auto_start': False,
                    'shadow_assume_initial_depth': True,
                    'shadow_log_rate_hz': 2.0,
                    # Prevent close COCO stand-ins from entering HIT_BALL.
                    'ball_hit_distance_m': 0.05,
                    'hit_surge': 0.0,
                    'retreat_surge': 0.0,
                    'pass_surge': 0.0,
                },
            ],
        ),
        Node(
            package='rov_thruster_bridge',
            executable='thruster_bridge_node',
            namespace='auv',
            name='bench_rov_bridge',
            output='screen',
            parameters=[
                os.path.join(
                    bridge_share, 'config', 'thruster_params.yaml'),
                {
                    # Scale planner effort down to <= about 0.17 normalized.
                    'command_limits.max_surge': 400.0,
                    'command_limits.max_sway': 400.0,
                    'command_limits.max_heave': 1000.0,
                    # SEARCH_BALL yaw=8 maps to 0.267, just above the NanoPi
                    # horizontal input deadband (30/127).
                    'command_limits.max_yaw': 30.0,
                    'propeller_feedback_rate_hz': 10.0,
                },
            ],
            remappings=[
                ('control_command', '/auv/shadow/control_command'),
                ('motion_enable', '/auv/shadow/requested_motion_enable'),
                ('target_depth_m', '/auv/shadow/target_depth_m'),
            ],
        ),
    ])
