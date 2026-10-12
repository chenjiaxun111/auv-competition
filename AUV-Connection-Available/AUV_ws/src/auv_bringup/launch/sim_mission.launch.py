"""
Simulation mission launch: HoloOcean bridge + planner + PID tracker.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # HoloOcean -> ROS2 bridge (pose + camera + thruster)
        Node(
            package='holo_bridge',
            executable='holo_node',
            name='holo_bridge_node',
            output='screen',
        ),

        # Planner + PID (shared)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'common_planner.launch.py')),
        ),
    ])
