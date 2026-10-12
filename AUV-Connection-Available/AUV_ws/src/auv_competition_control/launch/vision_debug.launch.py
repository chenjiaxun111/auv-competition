"""Camera + dual models + browser ONLY; no controller or actuator bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('auv_competition_control'),
                          'config', 'competition_params.yaml')
    actions = [
        DeclareLaunchArgument('target_ball_color', default_value='red'),
        DeclareLaunchArgument('web_port', default_value='8081'),
        DeclareLaunchArgument('detector_model_path',
            default_value='/home/auv/jetson-zed-yolo/models/competition_latest.pt'),
        DeclareLaunchArgument('gate_pose_model_path',
            default_value='/home/auv/jetson-zed-yolo/models/gate_pose.pt'),
    ]
    nodes = [
        Node(package='auv_competition_control', executable='zed_sdk_source',
             name='zed_sdk_source', parameters=[params], output='screen'),
        Node(package='auv_competition_control', executable='dual_model_perception',
             name='dual_model_perception', output='screen', parameters=[params, {
                 'detector_model_path': ParameterValue(LaunchConfiguration('detector_model_path'), value_type=str),
                 'gate_pose_model_path': ParameterValue(LaunchConfiguration('gate_pose_model_path'), value_type=str),
                 'target_ball_color': LaunchConfiguration('target_ball_color'),
                 'vision_debug_all_models': True, 'publish_debug_image': True,
                 'debug_max_fps': 5.0,
             }], remappings=[
                 ('/auv/mission_state', '/auv/vision_debug/unused_state'),
                 ('/auv/perception/ball_observations', '/auv/vision_debug/ball_observations'),
                 ('/auv/perception/gate_pose_observations', '/auv/vision_debug/gate_pose_observations'),
             ]),
        Node(package='auv_competition_control', executable='perception_web_viewer',
             output='screen', parameters=[{'port': ParameterValue(
                 LaunchConfiguration('web_port'), value_type=int)}]),
    ]
    actions.extend(nodes)
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(
                reason='Vision debug process exited'))])))
    return LaunchDescription(actions)
