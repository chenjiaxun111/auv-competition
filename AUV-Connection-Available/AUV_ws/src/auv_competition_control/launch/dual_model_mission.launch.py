"""Launch state-aware dual-model perception and competition control.

The ZED ROS wrapper must already publish rectified left image, registered
depth, and camera info. Shadow mode is the safe default and starts no bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = get_package_share_directory('auv_competition_control')
    bridge_share = get_package_share_directory('rov_thruster_bridge')
    params = os.path.join(control_share, 'config', 'competition_params.yaml')
    shadow = LaunchConfiguration('shadow_mode')

    actions = [
        DeclareLaunchArgument('shadow_mode', default_value='true'),
        DeclareLaunchArgument(
            'start_zed_sdk_source', default_value='false',
            description='Open ZED directly and publish ROS RGB-D topics'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('assume_initial_depth', default_value='false'),
        DeclareLaunchArgument('target_ball_color', default_value='red'),
        DeclareLaunchArgument(
            'vertical_control_mode', default_value='visual_heave'),
        DeclareLaunchArgument(
            'detector_model_path',
            default_value='/home/auv/jetson-zed-yolo/models/competition.onnx'),
        DeclareLaunchArgument(
            'gate_pose_model_path',
            default_value='/home/auv/jetson-zed-yolo/models/gate_pose.pt'),
        DeclareLaunchArgument(
            'inference_device', default_value='0',
            description='Ultralytics device: 0 for CUDA GPU, cpu for fallback'),
        DeclareLaunchArgument(
            'use_half_precision', default_value='true',
            description='Use FP16 inference; set false for CPU fallback'),
        DeclareLaunchArgument(
            'publish_debug_image', default_value='false',
            description='Publish JPEG debug frames; disabled in competition for FPS'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed/zed_node/left/camera_info'),
        Node(
            package='auv_competition_control',
            executable='zed_sdk_source',
            name='zed_sdk_source',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_zed_sdk_source')),
            parameters=[params, {
                'image_topic': ParameterValue(
                    LaunchConfiguration('image_topic'), value_type=str),
                'depth_topic': ParameterValue(
                    LaunchConfiguration('depth_topic'), value_type=str),
                'camera_info_topic': ParameterValue(
                    LaunchConfiguration('camera_info_topic'), value_type=str),
            }],
        ),
        Node(
            package='auv_competition_control',
            executable='dual_model_perception',
            name='dual_model_perception',
            output='screen',
            parameters=[params, {
                'target_ball_color': LaunchConfiguration('target_ball_color'),
                'vertical_control_mode': ParameterValue(
                    LaunchConfiguration('vertical_control_mode'), value_type=str),
                'detector_model_path': ParameterValue(
                    LaunchConfiguration('detector_model_path'), value_type=str),
                'gate_pose_model_path': ParameterValue(
                    LaunchConfiguration('gate_pose_model_path'), value_type=str),
                'inference_device': ParameterValue(
                    LaunchConfiguration('inference_device'), value_type=str),
                'use_half_precision': ParameterValue(
                    LaunchConfiguration('use_half_precision'), value_type=bool),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'), value_type=bool),
                'image_topic': ParameterValue(
                    LaunchConfiguration('image_topic'), value_type=str),
                'depth_topic': ParameterValue(
                    LaunchConfiguration('depth_topic'), value_type=str),
                'camera_info_topic': ParameterValue(
                    LaunchConfiguration('camera_info_topic'), value_type=str),
            }],
        ),
        Node(
            package='auv_competition_control',
            executable='competition_controller',
            name='competition_controller',
            output='screen',
            parameters=[params, {
                'target_ball_color': LaunchConfiguration('target_ball_color'),
                'shadow_mode': ParameterValue(shadow, value_type=bool),
                'vertical_control_mode': ParameterValue(
                    LaunchConfiguration('vertical_control_mode'), value_type=str),
                'shadow_auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'shadow_assume_initial_depth': ParameterValue(
                    LaunchConfiguration('assume_initial_depth'), value_type=bool),
                'use_legacy_objects3d': False,
            }],
        ),
        Node(
            package='rov_thruster_bridge',
            executable='thruster_bridge_node',
            namespace='auv',
            name='rov_bridge',
            output='screen',
            condition=UnlessCondition(shadow),
            parameters=[os.path.join(
                bridge_share, 'config', 'thruster_params.yaml')],
        ),
    ]
    # Any critical process exit shuts down the chain; no automatic re-arming.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[EmitEvent(event=Shutdown(reason='Critical mission node exited'))],
    )) for action in actions if isinstance(action, Node)]
    return LaunchDescription(handlers + actions)
