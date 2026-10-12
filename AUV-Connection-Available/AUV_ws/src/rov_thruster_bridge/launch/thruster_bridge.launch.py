import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('rov_thruster_bridge')
    config = os.path.join(pkg_dir, 'config', 'thruster_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rov_host', default_value='192.168.137.219'),
        DeclareLaunchArgument('rov_port', default_value='8888'),
        Node(
            package='rov_thruster_bridge',
            executable='thruster_bridge_node',
            namespace='auv',
            name='rov_bridge',
            parameters=[config, {
                'rov_host': LaunchConfiguration('rov_host'),
                'rov_port': ParameterValue(
                    LaunchConfiguration('rov_port'), value_type=int),
            }],
            output='screen',
        ),
    ])
