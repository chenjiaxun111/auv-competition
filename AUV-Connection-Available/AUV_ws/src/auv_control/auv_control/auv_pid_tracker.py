#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from tf2_ros import TransformException, Buffer, TransformListener
import math

class AuvPidTracker(Node):
    def __init__(self):
        super().__init__('auv_pid_tracker')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Wrench, '/auv/control_command', 10)

        # 平面控制 (Surge & Yaw)
        self.kp_dist = 10.0
        self.kd_dist = 5.0
        self.kp_yaw = 23.0
        self.kd_yaw = 5.0

        # 【3D 核心】垂直方向控制 (Heave)
        self.kp_z = 15.0  
        self.kd_z = 15.0

        self.prev_dist_error = 0.0
        self.prev_yaw_error = 0.0
        self.prev_z_error = 0.0
        self.is_reached = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🌊 3D PID 追踪节点已启动，正在监听三维 dummy_target...")

    def control_loop(self):
        try:
            now = rclpy.time.Time()
            auv_global = self.tf_buffer.lookup_transform('map', 'base_link', now)
            target_local = self.tf_buffer.lookup_transform('base_link', 'dummy_target', now)
        except TransformException:
            return 

        auv_x = auv_global.transform.translation.x
        auv_y = auv_global.transform.translation.y
        auv_z = auv_global.transform.translation.z

        dx = target_local.transform.translation.x
        dy = target_local.transform.translation.y
        dz = target_local.transform.translation.z # 获取 Z 轴相对误差

        dist_error = math.sqrt(dx**2 + dy**2)
        yaw_error = math.atan2(dy, dx)

        # 📊 3D 版专属监控日志
        self.get_logger().info(
            f'🌍 位置: X={auv_x:.1f}, Y={auv_y:.1f}, Z={auv_z:.1f} | '
            f'🎯 平面距目标: {dist_error:.2f}m | '
            f'📉 深度差: {dz:.2f}m | '
            f'📐 角度差: {math.degrees(yaw_error):.1f}°'
        )

        # PID 计算
        surge_cmd = self.kp_dist * dist_error + self.kd_dist * (dist_error - self.prev_dist_error)
        yaw_cmd = self.kp_yaw * yaw_error + self.kd_yaw * (yaw_error - self.prev_yaw_error)
        heave_cmd = self.kp_z * dz + self.kd_z * (dz - self.prev_z_error) # 计算垂直推力

        # 限幅
        surge_cmd = max(min(surge_cmd, 100.0), -50.0)
        yaw_cmd = max(min(yaw_cmd, 30.0), -30.0)
        heave_cmd = max(min(heave_cmd, 250.0), -250.0) 

        # 【3D 停靠判定】必须 平面距离 < 0.8m 且 深度误差 < 0.5m 才算到达
        if dist_error < 0.8 and abs(dz) < 0.5:
            surge_cmd, yaw_cmd = 0.0, 0.0
            if not self.is_reached:
                self.get_logger().info('\n🎉 追踪成功！AUV 已平稳停靠在 3D 目标点！\n')
                self.is_reached = True
        else:
            if self.is_reached:
                self.get_logger().info('\n⚓ 目标点已更新，AUV 重新起航下潜！\n')
                self.is_reached = False

        # 发布包含垂直推力的 3D 指令
        msg = Wrench()
        msg.force.x = float(surge_cmd)
        msg.force.z = float(heave_cmd) 
        msg.torque.z = float(yaw_cmd)
        self.cmd_pub.publish(msg)

        self.prev_dist_error = dist_error
        self.prev_yaw_error = yaw_error
        self.prev_z_error = dz

def main(args=None):
    rclpy.init(args=args)
    node = AuvPidTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()