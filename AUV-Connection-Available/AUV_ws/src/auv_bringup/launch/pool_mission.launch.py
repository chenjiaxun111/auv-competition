"""
Real-pool mission launch: vrpn_mocap + Nokov TF bridge + planner + PID + ROV thruster.

Prerequisites:
  1. XINGYING 动捕软件已启动，VRPN 广播已开启
  2. ROV 已上电，网络可达 (默认 192.168.137.219:8888)
  3. 刚体名称在 XINGYING 中配置为 "ROV"

Usage:
  ros2 launch auv_bringup pool_mission.launch.py
  ros2 launch auv_bringup pool_mission.launch.py \
      server:=10.1.1.198 \
      tracker_name:=ROV \
      rov_host:=192.168.137.219
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nokov_dir = get_package_share_directory('nokov_bridge')
    thruster_dir = get_package_share_directory('rov_thruster_bridge')
    bringup_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # --- VRPN 服务器 ---
        DeclareLaunchArgument('server', default_value='10.1.1.198',
                              description='XINGYING VRPN 服务器 IP'),

        # --- Tracker 名称 ---
        DeclareLaunchArgument('tracker_name', default_value='ROV'),

        # --- Nokov 全链路 (vrpn_mocap + TF bridge) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nokov_dir, 'launch', 'nokov_bridge.launch.py')),
            launch_arguments={
                'server': '10.1.1.198',
                'tracker_name': 'ROV',
            }.items(),
        ),

        # --- 规划器 + PID (共享) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'common_planner.launch.py')),
        ),

        # --- Wrench → 6ch PWM → ROV JSON-RPC ---
        Node(
            package='rov_thruster_bridge',
            executable='thruster_bridge_node',
            name='thruster_bridge_node',
            parameters=[os.path.join(thruster_dir, 'config', 'thruster_params.yaml')],
            output='screen',
        ),
    ])
