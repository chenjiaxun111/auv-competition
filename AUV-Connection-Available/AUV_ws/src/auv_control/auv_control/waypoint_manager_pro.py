#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import os
from datetime import datetime

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager_pro')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.start_pos = None     
        self.initialized = False  
        
        self.current_wp_idx = 0
        self.reach_threshold = 1.0 # 3D 距离到达判定阈值
        
        self.x_history = []
        self.y_history = []
        self.z_history = [] 

        self.tasks_completed = False

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("🗺️ 3D 航点大脑正在初始化，等待获取 AUV 实时位置以生成螺旋轨迹...")

    def generate_spiral_path(self, start_pos):
        """生成 3D 螺旋轨迹（自适应上升或下潜）"""
        path = []
        self.start_pos = start_pos

        # ================= 螺旋线核心参数 =================
        z_start = start_pos[2]
        
        # 智能判定：如果起点在深水区(<-50m)，则向上螺旋爬升到 -5m
        # 如果起点在浅水区(>-50m)，则向下螺旋潜水到 -50m
        if z_start < -50.0:
            z_end = -5.0
        else:
            z_end = -50.0
            
        radius = 15.0             # 盘旋半径：15 米
        num_turns = 3.0           # 盘旋圈数：3 圈
        num_points = 120           # 航点总数：60 个点画出圆滑曲线
        # ==================================================

        # 设定圆心：让 AUV 从当前位置直接平滑切入圆周
        center_x = start_pos[0] + radius
        center_y = start_pos[1]

        for i in range(num_points + 1):
            t = i / num_points
            theta = t * num_turns * 2 * math.pi
            
            # 计算 Z：随进度线性变化
            curr_z = z_start + t * (z_end - z_start)
            
            # 计算 X, Y：圆的参数方程 (加 pi 是为了让起点正好等于 start_pos)
            curr_x = center_x + radius * math.cos(theta + math.pi)
            curr_y = center_y + radius * math.sin(theta + math.pi)
            
            path.append((curr_x, curr_y, curr_z))
        
        self.waypoints = path
        self.initialized = True
        self.get_logger().info(f"🌀 3D 螺旋轨迹生成完毕！起点深度 {z_start:.1f}m ➔ 目标深度 {z_end:.1f}m。共 {len(self.waypoints)} 个精细路点。")

    def loop(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            
            self.x_history.append(curr_x)
            self.y_history.append(curr_y)
            self.z_history.append(curr_z)
            
            if not self.initialized:
                self.generate_spiral_path((curr_x, curr_y, curr_z))
                return
        except Exception:
            return

        if self.current_wp_idx >= len(self.waypoints):
            if not self.tasks_completed:
                self.get_logger().info("🎉 3D 螺旋航行任务圆满完成！AUV 正在终点平稳悬停。")
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

        # 3D 空间欧氏距离判定
        dist = math.sqrt((curr_x - target[0])**2 + (curr_y - target[1])**2 + (curr_z - target[2])**2)
        
        if dist < self.reach_threshold:
            self.get_logger().info(f"✅ 螺旋进度 [{self.current_wp_idx + 1}/{len(self.waypoints)}] | 3D 坐标: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
            self.current_wp_idx += 1

    def save_mission_report(self):
        """生成符合学术论文标准的 3D 螺旋空间轨迹报告图"""
        if not self.x_history:
            print("⚠️ 无数据，未生成图表。")
            return
            
        print("\n📊 正在渲染学术级 3D 轨迹报告图...")
        
        # --- 1. 学术全局样式配置 ---
        plt.rcParams.update({
            "font.family": "serif",       
            "font.serif": ["Times New Roman"], 
            "font.size": 12,               
            "axes.labelsize": 14,          
            "legend.fontsize": 11,         
            "xtick.labelsize": 12,
            "ytick.labelsize": 12
        })

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # --- 2. 绘制轨迹与航点 ---
        # 实际路径：深蓝色加粗实线
        ax.plot(self.x_history, self.y_history, self.z_history, 
                color='#1f77b4', linestyle='-', linewidth=2.5, 
                label='AUV Actual Trajectory', zorder=5)
        
        # 规划航点：浅红点，半透明处理，突出实际轨迹
        wp_x = [wp[0] for wp in self.waypoints]
        wp_y = [wp[1] for wp in self.waypoints]
        wp_z = [wp[2] for wp in self.waypoints]
        ax.scatter(wp_x, wp_y, wp_z, 
                   color='#d62728', marker='.', s=25, alpha=0.3, 
                   label='Planned Waypoints', zorder=3)
        
        # --- 3. 增强空间感：XY平面投影 ---
        if self.z_history and wp_z:
            min_z = min(min(self.z_history), min(wp_z)) - 2.0
            ax.plot(self.x_history, self.y_history, [min_z]*len(self.z_history), 
                    color='gray', linestyle='--', alpha=0.4, linewidth=1, label='XY Projection')

        # --- 4. 关键位置标注 ---
        if self.start_pos:
            ax.scatter(self.start_pos[0], self.start_pos[1], self.start_pos[2], 
                       color='#2ca02c', marker='o', s=150, edgecolors='k', label='Start Position', zorder=10)
        
        # 终点使用五角星标注
        if self.x_history:
            ax.scatter(self.x_history[-1], self.y_history[-1], self.z_history[-1], 
                       color='#ff7f0e', marker='*', s=200, edgecolors='k', label='End Position', zorder=10)

        # --- 5. 坐标轴范围与视角优化 (解决留白问题) ---
        if self.z_history and wp_z:
            z_data = self.z_history + wp_z
            z_min, z_max = min(z_data), max(z_data)
            ax.set_zlim(z_min - 5, z_max + 5) # 紧凑显示
        
        ax.set_title('3D Helical Trajectory Tracking Performance', fontsize=16, pad=20)
        ax.set_xlabel('X Coordinate (m)')
        ax.set_ylabel('Y Coordinate (m)')
        ax.set_zlabel('Depth (m)')
        
        # 设置最佳观察视角
        ax.view_init(elev=22, azim=-40)
        
        # 图例美化
        ax.legend(loc='upper right', frameon=True, fancybox=False, edgecolor='black')
        ax.grid(True, linestyle=':', alpha=0.6)
        
        # --- 6. 自动保存多格式 ---
        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M")
        pdf_path = f"AUV_3D_Spiral_Report_{now_str}.pdf"
        png_path = f"AUV_3D_Spiral_Report_{now_str}.png"
        
        plt.savefig(pdf_path, bbox_inches='tight') # 矢量图用于 LaTeX
        plt.savefig(png_path, dpi=300, bbox_inches='tight') # 高清位图用于 Word/PPT
        
        print(f"✅ 学术级图表已生成:\n   - 矢量图(LaTeX建议): {os.getcwd()}/{pdf_path}\n   - 高清图: {os.getcwd()}/{png_path}")
        plt.show()

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