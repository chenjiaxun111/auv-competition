"""
Common planner + PID tracker launch (shared between sim and pool modes).
Launches the path planner and PID tracking node.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    auv_control_dir = os.path.join(
        get_package_share_directory('nokov_bridge'), '..', '..', '..',
        'src', 'auv_control', 'auv_control')

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='hybrid',
                              description='Planner type: hybrid, waypoint, or spiral'),

        ExecuteProcess(
            cmd=['python3', os.path.join(auv_control_dir, 'hybrid_planner_node.py')],
            name='hybrid_planner',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['python3', os.path.join(auv_control_dir, 'auv_pid_tracker.py')],
            name='auv_pid_tracker',
            output='screen',
        ),
    ])
