#!/usr/bin/env python3
"""
Nokov/XINGYING 动捕数据诊断工具
===============================
在连接动捕系统之前，先用这个脚本探查：
1. vrpn_client_ros2 发布了哪些 topic
2. PoseStamped 数据的单位和坐标系方向
3. 帮助判断是 mm 还是 m、Z轴朝上还是朝下

用法:
  # 先确保 vrpn_client_ros2 在运行
  ros2 run nokov_bridge diagnose_nokov

  # 也可以指定监听的 topic 前缀
  ros2 run nokov_bridge diagnose_nokov --ros-args -p topic_filter:="/vrpn"

输出会持续打印接收到的位姿数据，帮助你判断：
  - 数值量级 (mm 级 ≈ 几千, m 级 ≈ 几)
  - 坐标系方向 (移动机器人看数值变化)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math


class NokovDiagnostic(Node):
    def __init__(self):
        super().__init__('nokov_diagnostic')

        self.declare_parameter('topic_filter', '')
        self.topic_filter = self.get_parameter('topic_filter').value

        # 记录每个 topic 的历史数据用于分析
        self.history = {}  # topic -> list of (x, y, z)

        # 先列出所有话题
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔍 Nokov/XINGYING 动捕数据诊断工具已启动')
        self.get_logger().info('=' * 60)

        # 定时器: 每 3 秒列出一次话题 (前几次)
        self.list_count = 0
        self.topic_list_timer = self.create_timer(3.0, self.list_topics)
        self.subscribe_timer = self.create_timer(5.0, self.auto_subscribe)

        self.subscribed = False

    def list_topics(self):
        """列出所有话题, 寻找 vrpn/nokov/pose 相关的话题"""
        self.list_count += 1

        topic_names = self.get_topic_names_and_types()

        # 筛选感兴趣的话题
        interesting = []
        for name, types in topic_names:
            name_lower = name.lower()
            if any(kw in name_lower for kw in ['vrpn', 'nokov', 'pose', 'tracker', 'rigid', 'mocap', 'motion']):
                interesting.append((name, types))

        if interesting:
            self.get_logger().info(f'\n📡 发现 {len(interesting)} 个相关话题:')
            for name, types in interesting:
                type_str = ', '.join(types)
                self.get_logger().info(f'   📍 {name}')
                self.get_logger().info(f'      类型: {type_str}')
        else:
            self.get_logger().info(f'\n⚠️  未发现动捕相关话题 (已搜索 {self.list_count} 次)')
            self.get_logger().info(f'   当前话题总数: {len(topic_names)}')
            if self.list_count >= 3:
                self.get_logger().info('   提示: 请确认 vrpn_client_ros2 是否在运行')
                self.get_logger().info('   启动命令: ros2 run vrpn_client_ros2 vrpn_client_ros2 --ros-args -p server:=<XINGYING_IP>')

        # 只列前 5 次
        if self.list_count >= 10:
            self.topic_list_timer.cancel()

    def auto_subscribe(self):
        """自动发现并订阅相关话题"""
        if self.subscribed:
            return

        topic_names = self.get_topic_names_and_types()
        pose_topics = []

        for name, types in topic_names:
            type_set = [t.lower() for t in types]
            for t in type_set:
                if 'posestamped' in t or 'geometry_msgs/msg/posestamped' in t:
                    name_lower = name.lower()
                    if self.topic_filter and self.topic_filter not in name_lower:
                        continue
                    if any(kw in name_lower for kw in ['vrpn', 'nokov', 'pose', 'tracker', 'rigid', 'mocap']):
                        pose_topics.append(name)
                        break
                elif 'odometry' in t:
                    name_lower = name.lower()
                    if self.topic_filter and self.topic_filter not in name_lower:
                        continue
                    if any(kw in name_lower for kw in ['vrpn', 'nokov', 'tracker', 'rigid', 'mocap']):
                        pose_topics.append(name)
                        break
                elif 'tfmessage' in t or 'tf2_msgs/msg/tfmessage' in t:
                    name_lower = name.lower()
                    if name == '/tf' or name == '/tf_static':
                        pass  # skip default tf topics
                    elif self.topic_filter and self.topic_filter not in name_lower:
                        continue
                    else:
                        pose_topics.append(name)
                        break

        if pose_topics:
            self.get_logger().info(f'\n🎯 自动订阅 {len(pose_topics)} 个话题...')
            for topic in pose_topics:
                self._subscribe_to(topic)
            self.subscribed = True
            self.subscribe_timer.cancel()

            self.get_logger().info('\n📊 开始接收数据，移动机器人或刚体查看数值变化...')
            self.get_logger().info('   提示: Ctrl+C 退出时会打印分析总结\n')

    def _subscribe_to(self, topic: str):
        """动态订阅话题"""
        topic_names = self.get_topic_names_and_types()
        types = topic_names.get(topic, [])

        for t in types:
            t_lower = t.lower()
            if 'posestamped' in t_lower:
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, t=topic: self.pose_callback(t, msg),
                    10)
                self.get_logger().info(f'   ✅ 已订阅 PoseStamped: {topic}')
                self.history[topic] = []
                return
            elif 'odometry' in t_lower:
                self.create_subscription(
                    Odometry, topic,
                    lambda msg, t=topic: self.odom_callback(t, msg),
                    10)
                self.get_logger().info(f'   ✅ 已订阅 Odometry: {topic}')
                self.history[topic] = []
                return
            elif 'tfmessage' in t_lower:
                self.create_subscription(
                    TFMessage, topic,
                    lambda msg, t=topic: self.tf_callback(t, msg),
                    100)
                self.get_logger().info(f'   ✅ 已订阅 TFMessage: {topic}')
                return

        self.get_logger().warn(f'   ⚠️  无法确定 {topic} 的消息类型: {types}')

    def pose_callback(self, topic: str, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        # 保存历史
        if topic in self.history:
            self.history[topic].append((p.x, p.y, p.z))
            if len(self.history[topic]) > 100:
                self.history[topic] = self.history[topic][-50:]

        # 计算欧拉角 (roll, pitch, yaw)
        roll, pitch, yaw = self._euler_from_quaternion(q.x, q.y, q.z, q.w)

        # 范数帮助判断单位
        norm = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        unit_guess = ''
        if norm > 500:
            unit_guess = '🔴 疑似 mm (数值很大)'
        elif norm > 100:
            unit_guess = '🟡 数值偏大，不确定单位'
        else:
            unit_guess = '🟢 疑似 m (数值合理)'

        self.get_logger().info(
            f'\n📍 [{topic}] PoseStamped (frame: {msg.header.frame_id})'
            f'\n   位置: X={p.x:+.4f}  Y={p.y:+.4f}  Z={p.z:+.4f}'
            f'\n   范数: |pos| = {norm:.2f}  → {unit_guess}'
            f'\n   姿态: Roll={math.degrees(roll):+.1f}°  Pitch={math.degrees(pitch):+.1f}°  Yaw={math.degrees(yaw):+.1f}°'
            f'\n   四元数: x={q.x:.4f} y={q.y:.4f} z={q.z:.4f} w={q.w:.4f}'
        )

    def odom_callback(self, topic: str, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._euler_from_quaternion(q.x, q.y, q.z, q.w)
        norm = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        self.get_logger().info(
            f'\n📍 [{topic}] Odometry'
            f'\n   header.frame_id: {msg.header.frame_id}'
            f'\n   child_frame_id:  {msg.child_frame_id}'
            f'\n   位置: X={p.x:+.4f}  Y={p.y:+.4f}  Z={p.z:+.4f}  |pos|={norm:.2f}'
            f'\n   姿态: Roll={math.degrees(roll):+.1f}°  Pitch={math.degrees(pitch):+.1f}°  Yaw={math.degrees(yaw):+.1f}°'
            f'\n   线速度: vx={msg.twist.twist.linear.x:.3f} vy={msg.twist.twist.linear.y:.3f} vz={msg.twist.twist.linear.z:.3f}'
        )

    def tf_callback(self, topic: str, msg: TFMessage):
        for trans in msg.transforms:
            t = trans.transform.translation
            self.get_logger().info(
                f'\n🔗 [{topic}] TF: {trans.header.frame_id} → {trans.child_frame_id}'
                f'\n   位移: X={t.x:+.4f}  Y={t.y:+.4f}  Z={t.z:+.4f}'
            )

    def _euler_from_quaternion(self, x, y, z, w):
        """四元数 → 欧拉角 (roll, pitch, yaw)"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = NokovDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 打印分析总结
        node.get_logger().info('\n' + '=' * 60)
        node.get_logger().info('📊 分析总结')
        node.get_logger().info('=' * 60)

        for topic, history in node.history.items():
            if len(history) < 2:
                node.get_logger().info(f'{topic}: 数据不足 (仅 {len(history)} 条)')
                continue

            # 分析数值范围
            xs = [h[0] for h in history]
            ys = [h[1] for h in history]
            zs = [h[2] for h in history]

            x_range = max(xs) - min(xs)
            y_range = max(ys) - min(ys)
            z_range = max(zs) - min(zs)

            avg_mag = sum(math.sqrt(x**2 + y**2 + z**2) for x, y, z in history) / len(history)

            node.get_logger().info(f'\n📍 {topic}:')
            node.get_logger().info(f'   数据条数: {len(history)}')
            node.get_logger().info(f'   X 范围: [{min(xs):.2f}, {max(xs):.2f}] 跨度={x_range:.2f}')
            node.get_logger().info(f'   Y 范围: [{min(ys):.2f}, {max(ys):.2f}] 跨度={y_range:.2f}')
            node.get_logger().info(f'   Z 范围: [{min(zs):.2f}, {max(zs):.2f}] 跨度={z_range:.2f}')
            node.get_logger().info(f'   平均距离原点: {avg_mag:.2f}')

            # 单位判断
            if avg_mag > 500:
                node.get_logger().warn(f'   ⚠️  单位判断: 很可能是 **毫米(mm)**，需要在 bridge 中 ×0.001 转米')
            elif avg_mag > 50:
                node.get_logger().warn(f'   ⚠️  单位判断: 可能是 cm 或 dm，请根据水池实际尺寸判断')
            else:
                node.get_logger().info(f'   ✅ 单位判断: 很可能是 **米(m)**，可直接使用')

            # Z轴方向判断
            if z_range > 0.5:
                z_trend = '增大' if zs[-1] > zs[0] else '减小'
                node.get_logger().info(f'   🔍 Z 轴趋势: {z_trend} (范围 {z_range:.2f})')
                node.get_logger().info(f'   💡 提示: 把机器人放在水面，如果 Z 为正值且是最大高度 → Z-up (需翻转)')
                node.get_logger().info(f'           如果 Z 为负值(越深越负) → 已是深度方向，直接可用')

        node.get_logger().info('\n💡 下一步: 根据上述信息修改 nokov_tf_bridge.py 的配置')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
