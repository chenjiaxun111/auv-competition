"""Nokov 全链路启动: vrpn_mocap + TF bridge.

用法:
  ros2 launch nokov_bridge nokov_bridge.launch.py
  ros2 launch nokov_bridge nokov_bridge.launch.py \
      tracker_name:=ROV \
      server:=10.1.1.198 \
      invert_z:=true \
      offset_x:=1.0 offset_y:=2.0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('nokov_bridge')
    config = os.path.join(pkg_dir, 'config', 'nokov_params.yaml')

    return LaunchDescription([
        # --- vrpn_mocap 参数 ---
        DeclareLaunchArgument('server', default_value='10.1.1.198',
                              description='XINGYING VRPN 服务器 IP'),
        DeclareLaunchArgument('port', default_value='3883',
                              description='VRPN 端口'),
        DeclareLaunchArgument('enable_vrpn', default_value='true',
                              description='是否启动 vrpn_mocap 客户端'),

        # --- nokov_tf_bridge 参数 ---
        DeclareLaunchArgument('tracker_name', default_value='ROV'),
        DeclareLaunchArgument('invert_z', default_value='false'),

        # --- vrpn_mocap 客户端 ---
        Node(
            package='vrpn_mocap',
            executable='client_node',
            namespace='vrpn_mocap',
            name='vrpn_client',
            parameters=[{
                'server': LaunchConfiguration('server'),
                'port': LaunchConfiguration('port'),
                'frame_id': 'world',
                'update_freq': 100.0,
                'refresh_freq': 1.0,
                'multi_sensor': True,
                'use_vrpn_timestamps': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_vrpn')),
            output='screen',
        ),

        # --- Nokov Pose → TF bridge ---
        Node(
            package='nokov_bridge',
            executable='nokov_tf_bridge',
            name='nokov_tf_bridge',
            parameters=[
                config,
                {
                    'tracker_name': LaunchConfiguration('tracker_name'),
                    'invert_z': LaunchConfiguration('invert_z'),
                },
            ],
            output='screen',
        ),
    ])
