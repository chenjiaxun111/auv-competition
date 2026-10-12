#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PointStamped, Wrench
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from cv_bridge import CvBridge

# 引入通信与解析库用于全息渲染
from std_msgs.msg import String
import json

import holoocean
import cv2
import numpy as np
import time

# 引入画图库用于记录推进器推力曲线
import matplotlib.pyplot as plt
import os
from datetime import datetime

class HoloOceanBridge(Node):
    def __init__(self):
        super().__init__('holoocean_bridge_node')
        
        self.image_pub = self.create_publisher(Image, '/auv/camera/image_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        self.control_sub = self.create_subscription(
            Wrench,
            '/auv/control_command',
            self.control_callback,
            10)
        
        # ================= 全息投影监听器 =================
        self.drawn_obstacles = False
        self.obs_sub = self.create_subscription(
            String, 
            '/holoocean_obstacles', 
            self.obs_callback, 
            10)
        # ================================================

        self.cv_bridge = CvBridge()
        self.publish_static_transform()
        
        self.get_logger().info('正在启动 HoloOcean (RTX 5090 模式)...')
        
        # ================= 自定义出生点 =================
        scenario_cfg = holoocean.packagemanager.get_scenario("OpenWater-HoveringCamera")
        scenario_cfg['package_name'] = 'Ocean' 
        scenario_cfg['agents'][0]['location'] = [35.0, -40.0, -5.0] 
        self.env = holoocean.make(scenario_cfg=scenario_cfg)
        # ================================================
        
        self.env.reset()
        self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # ================= 推进器负荷记录仪 =================
        self.start_time_log = time.time()
        self.time_history = []
        self.thruster_history = {i: [] for i in range(8)} 
        self.get_logger().info("📊 8 通道推进器负荷数据记录仪已启动...")
        # ==================================================

        self.timer = self.create_timer(0.05, self.timer_callback)

    # ================= HoloOcean 实体绘制引擎 =================
    def obs_callback(self, msg):
        """接收大脑传来的 JSON 坐标，在 Unreal Engine 中生成视觉实体"""
        if self.drawn_obstacles or self.env is None:
            return 
            
        try:
            obs_data = json.loads(msg.data)
            
            # 1. 渲染复杂的 L 型峡谷与沉船 (Box)
            for b in obs_data.get("boxes", []):
                x_min, x_max, y_min, y_max, z_min, z_max = b
                center = [(x_max + x_min) / 2.0, (y_max + y_min) / 2.0, (z_max + z_min) / 2.0]
                extent = [(x_max - x_min) / 2.0, (y_max - y_min) / 2.0, (z_max - z_min) / 2.0]
                self.env.draw_box(center, extent, color=[150, 150, 150], thickness=8.0, lifetime=0.0)

            # 2. 渲染静态暗礁/水雷 (【已修复】：用 Box 代替 Sphere 渲染警戒区域)
            for s in obs_data.get("spheres", []):
                cx, cy, cz, r = s
                # 画出橙色的正方体框架，完美等效球体的危险包围盒 (AABB)
                self.env.draw_box([cx, cy, cz], [r, r, r], color=[255, 165, 0], thickness=8.0, lifetime=0.0)

            self.drawn_obstacles = True
            self.get_logger().info("✨ 全息投影完成！成功在 HoloOcean 中生成 3D 视觉实体框架！")
            
        except Exception as e:
            self.get_logger().error(f"❌ 渲染障碍物失败: {e}")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def control_callback(self, msg):
        surge = msg.force.x       
        heave = msg.force.z       
        yaw_torque = msg.torque.z 
        
        self.action[0] = -heave
        self.action[1] = -heave
        self.action[2] = -heave
        self.action[3] = -heave
        
        self.action[4] = surge - yaw_torque  
        self.action[5] = surge + yaw_torque  
        self.action[6] = surge - yaw_torque  
        self.action[7] = surge - yaw_torque  

    def matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            s = 0.5 / np.sqrt(tr + 1.0); qw = 0.25 / s
            qx = (R[2,1] - R[1,2]) * s; qy = (R[0,2] - R[2,0]) * s; qz = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                qw = (R[2,1] - R[1,2]) / s; qx = 0.25 * s; qy = (R[0,1] + R[1,0]) / s; qz = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                qw = (R[0,2] - R[2,0]) / s; qx = (R[0,1] + R[1,0]) / s; qy = 0.25 * s; qz = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                qw = (R[1,0] - R[0,1]) / s; qx = (R[0,2] + R[2,0]) / s; qy = (R[1,2] + R[2,1]) / s; qz = 0.25 * s
        return [qx, qy, qz, qw]

    def timer_callback(self):
        # ================= 记录当前时间戳与电机的实时指令 =================
        current_t = time.time() - self.start_time_log
        self.time_history.append(current_t)
        for i in range(8):
            self.thruster_history[i].append(self.action[i])
        # ====================================================================

        state = self.env.step(self.action)

        if 'PoseSensor' in state:
            pose = state['PoseSensor']
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(pose[0, 3])
            t.transform.translation.y = float(pose[1, 3])
            t.transform.translation.z = float(pose[2, 3])
            q = self.matrix_to_quaternion(pose[0:3, 0:3])
            t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            self.tf_broadcaster.sendTransform(t)
            
        if 'LeftCamera' in state:
            image_rgba = state['LeftCamera']
            image_bgr = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2BGR)
            ros_image_msg = self.cv_bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            self.image_pub.publish(ros_image_msg)

    # ================= 生成推进器负荷曲线图 (8通道独立分离版) =================
    def save_thruster_report(self):
        if not self.time_history: return
        self.get_logger().info("\n📊 正在渲染 [8 通道独立推进器负荷分析报告]...")
        
        # 建立 4行 x 2列 的子图阵列，统一共享 X 轴时间
        fig, axes = plt.subplots(4, 2, figsize=(18, 12), sharex=True)
        
        colors_v = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] # 垂直电机颜色
        colors_h = ['#9467bd', '#8c564b', '#e377c2', '#7f7f7f'] # 水平电机颜色

        for i in range(4):
            # === 左列：垂直推进器 (0, 1, 2, 3) ===
            ax_v = axes[i, 0]
            ax_v.plot(self.time_history, self.thruster_history[i], 
                      label=f'Thruster {i} (Vertical)', color=colors_v[i], alpha=0.9, linewidth=1.5)
            ax_v.set_ylabel('Output', fontsize=10)
            ax_v.grid(True, linestyle='--', alpha=0.6)
            ax_v.legend(loc='upper right')

            # === 右列：水平推进器 (4, 5, 6, 7) ===
            ax_h = axes[i, 1]
            ax_h.plot(self.time_history, self.thruster_history[i+4], 
                      label=f'Thruster {i+4} (Horizontal)', color=colors_h[i], alpha=0.9, linewidth=1.5)
            ax_h.set_ylabel('Output', fontsize=10)
            ax_h.grid(True, linestyle='--', alpha=0.6)
            ax_h.legend(loc='upper right')

        # 只在最底部的两个子图显示 X 轴标签
        axes[3, 0].set_xlabel('Mission Time (s)', fontsize=12, weight='bold')
        axes[3, 1].set_xlabel('Mission Time (s)', fontsize=12, weight='bold')

        # 在顶部的两个子图显示列标题
        axes[0, 0].set_title('Vertical Thrusters (Depth Control)', fontsize=14, weight='bold')
        axes[0, 1].set_title('Horizontal Thrusters (Evasion Control)', fontsize=14, weight='bold')

        plt.suptitle('AUV Individual Thruster Control Effort Analysis (Raw APF Baseline)', fontsize=18, weight='bold', y=0.98)
        
        # 自动调整布局，防止重叠
        plt.tight_layout()

        save_dir = "/home/cjx/cjx_project/AUV/AUV_ws/log/实验数据记录"
        if not os.path.exists(save_dir): os.makedirs(save_dir)
        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M")
        pdf_path = os.path.join(save_dir, f"AUV_Thrusters_Raw_{now_str}.pdf")
        
        plt.savefig(pdf_path, dpi=150, format='pdf', bbox_inches='tight')
        self.get_logger().info(f"💾 8通道独立推进器原始脉冲负荷报告已保存: {pdf_path}")
        plt.close(fig)

    def destroy_node(self):
        self.get_logger().info('正在关闭仿真环境并导出核心数据...')
        self.save_thruster_report()  # 确保在完全关闭前画图
        try: self.env.__exit__()
        except: self.env.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = HoloOceanBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()