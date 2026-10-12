#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 【新增】引入 3D 绘图工具包
import time
import math
import os
from datetime import datetime

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

# 1. 【适配 OpenWater】将坐标移回原点附近
        self.corners = [
            (0.0, 0.0, -225.0),    # 起跑区：浅水 5m
            (20.0, 0.0, -240.0),  # 顶点2：前进并下潜到 10m
            (20.0, 20.0, -250.0), # 顶点3：横移并下潜到 15m
            (0.0, 20.0, -235.0),  # 顶点4：后退并下潜到 20m (深水区)
            (0.0, 0.0, -225.0)     # 闭合：回到原点并爬升回 5m
        ]

        self.waypoints = []
        self.start_pos = None     
        self.initialized = False  
        
        self.current_wp_idx = 0
        self.reach_threshold = 1.0 
        
        self.x_history = []
        self.y_history = []
        self.z_history = [] # 【新增】记录 Z 轴轨迹历史

        self.tasks_completed = False

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("🗺️ 3D 航点大脑正在初始化，准备生成三维潜航路径...")

    def generate_interpolated_path(self, start_pos):
        """生成高密度路径：Z 轴也会自动进行线性插值计算"""
        path = []
        self.start_pos = start_pos

        num_start_steps = 20
        for j in range(num_start_steps + 1):
            frac = j / num_start_steps
            path.append((
                self.start_pos[0] + (self.corners[0][0] - self.start_pos[0]) * frac,
                self.start_pos[1] + (self.corners[0][1] - self.start_pos[1]) * frac,
                self.start_pos[2] + (self.corners[0][2] - self.start_pos[2]) * frac
            ))

        num_normal_steps = 4
        for i in range(len(self.corners) - 1):
            p1, p2 = self.corners[i], self.corners[i+1]
            for j in range(1, num_normal_steps + 2):
                frac = j / (num_normal_steps + 1)
                path.append((
                    p1[0] + (p2[0] - p1[0]) * frac,
                    p1[1] + (p2[1] - p1[1]) * frac,
                    p1[2] + (p2[2] - p1[2]) * frac
                ))
        
        self.waypoints = path
        self.initialized = True
        self.get_logger().info(f"🚀 3D 高精度路径生成完毕！共 {len(self.waypoints)} 个空间路点。")

    def loop(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            
            self.x_history.append(curr_x)
            self.y_history.append(curr_y)
            self.z_history.append(curr_z) # 【新增】记录当前深度
            
            if not self.initialized:
                self.generate_interpolated_path((curr_x, curr_y, curr_z))
                return
        except Exception:
            return

        if self.current_wp_idx >= len(self.waypoints):
            if not self.tasks_completed:
                self.get_logger().info("🎉 所有 3D 航点打卡成功！AUV 正在当前深度悬停。")
                self.tasks_completed = True
            return

        target = self.waypoints[self.current_wp_idx]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'dummy_target'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = target
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 【升维】计算 3D 空间距离 (加入了 Z 轴偏差)
        dist = math.sqrt((curr_x - target[0])**2 + (curr_y - target[1])**2 + (curr_z - target[2])**2)
        
        if dist < self.reach_threshold:
            self.get_logger().info(f"✅ 到达路点 [{self.current_wp_idx + 1}/{len(self.waypoints)}] | 3D 坐标: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
            self.current_wp_idx += 1

    def save_mission_report(self):
        """【升维】生成 3D 空间轨迹报告图"""
        if not self.x_history:
            print("⚠️ 无数据，未生成图表。")
            return
            
        print("\n📊 正在渲染 3D 空间任务报告图...")
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d') # 创建 3D 坐标系
        
        # 1. 3D 轨迹曲线
        ax.plot(self.x_history, self.y_history, self.z_history, 'b-', label='AUV Actual 3D Path', alpha=0.5)
        
        # 2. 3D 离散路点 (小红点)
        wp_x = [wp[0] for wp in self.waypoints]
        wp_y = [wp[1] for wp in self.waypoints]
        wp_z = [wp[2] for wp in self.waypoints]
        ax.scatter(wp_x, wp_y, wp_z, color='red', marker='.', s=30, label='3D Waypoints', alpha=0.6)
        
        # 3. 初始位置标识 (绿色大圆点)
        if self.start_pos:
            ax.scatter(self.start_pos[0], self.start_pos[1], self.start_pos[2], 
                       color='green', marker='o', s=180, label='Start Position', edgecolors='black', zorder=10)
            ax.text(self.start_pos[0], self.start_pos[1], self.start_pos[2], '  START', color='green', weight='bold')

        # 4. 关键顶点标识 (红色大星星)
        c_x = [c[0] for c in self.corners]
        c_y = [c[1] for c in self.corners]
        c_z = [c[2] for c in self.corners]
        ax.scatter(c_x, c_y, c_z, color='red', marker='*', s=200, label='Key 3D Targets', edgecolors='black', zorder=11)

        ax.set_title('AUV 3D Spatial Navigation Mission Report')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Depth (m)')
        #ax.invert_zaxis() # 【核心细节】翻转 Z 轴，使深水区在图像下方，符合物理直觉
        ax.legend(loc='upper right')
        
        # 保存：AUV_3D_mission_report_YYYY-MM-DD_HH-MM.png
        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M")
        filename = f"AUV_3D_mission_report_{now_str}.png"
        plt.savefig(filename)
        print(f"💾 3D 任务报告已保存: {os.getcwd()}/{filename}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_mission_report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()