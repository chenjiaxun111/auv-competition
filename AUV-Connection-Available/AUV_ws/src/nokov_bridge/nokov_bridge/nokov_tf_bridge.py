#!/usr/bin/env python3
"""
Nokov TF Bridge: 连接 vrpn_mocap → 广播 TF 变换链

TF 链: map → nokov_world → odom → base_link
  - map → nokov_world: 静态偏移 (offset_*)
  - nokov_world → odom: Nokov 动捕实时数据
  - odom → base_link: 单位变换 (恒等)

数据修正:
  - mm → m (scale=0.001)
  - Z-up (Nokov) → Z-down (ROS/水下惯例)  [可配置]

输入: /vrpn_mocap/{tracker_name}/pose{sensor_id} (PoseStamped)
输出: TF + /auv/pose + /auv/odom
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class NokovTfBridge(Node):
    def __init__(self):
        super().__init__('nokov_tf_bridge')

        # --- 参数声明 ---
        self.declare_parameter('tracker_name', 'ROV')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('vrpn_namespace', '/vrpn_mocap')

        self.declare_parameter('nokov_world_frame', 'nokov_world')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        # 单位与坐标系修正
        self.declare_parameter('scale', 0.001)       # mm → m
        self.declare_parameter('invert_z', True)      # Z-up → Z-down
        self.declare_parameter('invert_y', False)     # 如需翻转 Y 轴
        self.declare_parameter('swap_xy', False)      # 如需交换 XY

        # 静态偏移 (map 原点 相对 Nokov 原点)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('offset_roll', 0.0)
        self.declare_parameter('offset_pitch', 0.0)
        self.declare_parameter('offset_yaw', 0.0)

        # --- 读取参数 ---
        self.tracker_name = self.get_parameter('tracker_name').value
        self.sensor_id = self.get_parameter('sensor_id').value
        self.vrpn_namespace = self.get_parameter('vrpn_namespace').value

        self.nokov_world_frame = self.get_parameter('nokov_world_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.scale = self.get_parameter('scale').value
        self.invert_z = self.get_parameter('invert_z').value
        self.invert_y = self.get_parameter('invert_y').value
        self.swap_xy = self.get_parameter('swap_xy').value

        # --- TF 广播器 ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 辅助话题 ---
        self.pose_pub = self.create_publisher(PoseStamped, '/auv/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/auv/odom', 10)

        # --- 订阅 vrpn_mocap (BEST_EFFORT QoS, 匹配 vrpn_mocap 的 sensor_data) ---
        pose_topic = f'{self.vrpn_namespace}/{self.tracker_name}/pose{self.sensor_id}'
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, qos)

        # 缓存偏移参数 (每次回调都发布, 确保 TF 树中有)
        self._offset_x = self.get_parameter('offset_x').value
        self._offset_y = self.get_parameter('offset_y').value
        self._offset_z = self.get_parameter('offset_z').value
        self._offset_roll = self.get_parameter('offset_roll').value
        self._offset_pitch = self.get_parameter('offset_pitch').value
        self._offset_yaw = self.get_parameter('offset_yaw').value

        self.get_logger().info(
            f'✅ Nokov TF Bridge 就绪:'
            f'\n   订阅: {pose_topic}'
            f'\n   缩放: ×{self.scale} (mm→m)'
            f'\n   Z轴翻转: {self.invert_z}'
            f'\n   偏移: ({self._offset_x:.3f}, {self._offset_y:.3f}, {self._offset_z:.3f})m'
            f'\n   TF链: {self.map_frame} → {self.odom_frame} → {self.robot_frame}')

    def pose_callback(self, msg: PoseStamped):
        """接收 vrpn_mocap 的 PoseStamped，转换为 TF"""
        now = self.get_clock().now().to_msg()

        # --- 单位转换 + 坐标系修正 ---
        x = msg.pose.position.x * self.scale
        y = msg.pose.position.y * self.scale
        z = msg.pose.position.z * self.scale

        if self.swap_xy:
            x, y = y, x
        if self.invert_y:
            y = -y
        if self.invert_z:
            z = -z

        # --- 应用标定偏移: map坐标 = nokov坐标 + offset ---
        x += self._offset_x
        y += self._offset_y
        z += self._offset_z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # --- TF: map → odom (Nokov数据 + 偏移, 直接串联) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # --- TF: odom → base_link (identity, 满足 TF 树完整) ---
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = self.odom_frame
        t2.child_frame_id = self.robot_frame
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)

        # --- 辅助: /auv/pose ---
        pose_out = PoseStamped()
        pose_out.header.stamp = now
        pose_out.header.frame_id = self.map_frame
        pose_out.pose.position.x = x
        pose_out.pose.position.y = y
        pose_out.pose.position.z = z
        pose_out.pose.orientation.x = qx
        pose_out.pose.orientation.y = qy
        pose_out.pose.orientation.z = qz
        pose_out.pose.orientation.w = qw
        self.pose_pub.publish(pose_out)

        # --- 辅助: /auv/odom ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose = pose_out.pose
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = NokovTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
