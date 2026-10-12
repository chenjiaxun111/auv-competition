#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

# 绘图相关依赖
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.patches as patches                  
import numpy as np # 【新增】用于生成 3D 球体曲面网格

import math
import random
import os
from datetime import datetime

# ================= RRT 节点类 =================
class RRTNode:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

# ================= 航点管理器 =================
class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.start_pos = None     
        self.initialized = False  
        
        self.current_wp_idx = 0
        self.reach_threshold = 0.5 

        self.x_history = []
        self.y_history = []
        self.z_history = [] 

        self.tasks_completed = False

        self.goal_pos = (65.0, -10.0, -25.0) 
        self.bounds = {"x": (20, 80), "y": (-50, 0), "z": (-35, -4.0)} 
        
        # ================= 全新：非规则障碍物引擎 =================
        # 1. 复合异形障碍物 (由多个 AABB 互相穿插形成)
        # 格式: [X_min, X_max, Y_min, Y_max, Z_min, Z_max]
        self.box_obstacles = [
            # 这是一个“十字架”形状的异形残骸
            (38.0, 42.0, -22.0, -18.0, -25.0, -5.0), # 竖直的主柱子
            (35.0, 45.0, -21.0, -19.0, -17.0, -13.0) # 水平穿插的横梁
        ]
        
        # 2. 球体/圆形障碍物 (真正的非矩形，完美圆润)
        # 格式: (圆心_X, 圆心_Y, 圆心_Z, 半径_Radius)
        self.sphere_obstacles = [
            (50.0, -30.0, -15.0, 4.0),  # 圆石 1：位于中段的巨大圆球 (半径 4m)
            (58.0, -20.0, -20.0, 3.5),  # 悬浮水雷：偏右侧的圆形禁区 (半径 3.5m)
            (45.0, -15.0, -25.0, 3.0)   # 圆石 2：偏下的圆形礁石 (半径 3m)
        ]
        # ==========================================================

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("🗺️ 3D RRT 非规则地形规划器已启动，等待获取起点...")

    def check_collision(self, p1, p2):
        """【升级版碰撞引擎】同时支持检查矩形框和完美球体"""
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            cx = p1[0] + t * (p2[0] - p1[0])
            cy = p1[1] + t * (p2[1] - p1[1])
            cz = p1[2] + t * (p2[2] - p1[2])
            
            # 检查是否撞到矩形组
            for obs in self.box_obstacles:
                if (obs[0] <= cx <= obs[1] and 
                    obs[2] <= cy <= obs[3] and 
                    obs[4] <= cz <= obs[5]):
                    return True 
                    
            # 检查是否撞到球体 (计算采样点到圆心的三维欧几里得距离)
            for sphere in self.sphere_obstacles:
                dist_to_center = math.dist((cx, cy, cz), (sphere[0], sphere[1], sphere[2]))
                if dist_to_center <= sphere[3]: # 如果距离小于半径，说明扎进球里了
                    return True
                    
        return False

    def generate_rrt_path(self, start_pos):
        self.get_logger().info("🌳 正在计算 3D RRT 避障路径，请稍候...")
        self.start_pos = start_pos
        
        step_size = 2.0  
        max_iter = 3000  
        
        start_node = RRTNode(*start_pos)
        goal_node = RRTNode(*self.goal_pos)
        tree = [start_node]
        
        path_found = False
        
        for _ in range(max_iter):
            if random.random() < 0.1:
                rand_pos = self.goal_pos
            else:
                rand_pos = (
                    random.uniform(*self.bounds["x"]),
                    random.uniform(*self.bounds["y"]),
                    random.uniform(*self.bounds["z"])
                )
            
            nearest_node = min(tree, key=lambda n: math.dist((n.x, n.y, n.z), rand_pos))
            d = math.dist((nearest_node.x, nearest_node.y, nearest_node.z), rand_pos)
            if d == 0: continue
            
            step = min(step_size, d)
            new_x = nearest_node.x + step * (rand_pos[0] - nearest_node.x) / d
            new_y = nearest_node.y + step * (rand_pos[1] - nearest_node.y) / d
            new_z = nearest_node.z + step * (rand_pos[2] - nearest_node.z) / d
            
            if not self.check_collision((nearest_node.x, nearest_node.y, nearest_node.z), (new_x, new_y, new_z)):
                new_node = RRTNode(new_x, new_y, new_z)
                new_node.parent = nearest_node
                tree.append(new_node)
                
                if math.dist((new_node.x, new_node.y, new_node.z), self.goal_pos) < step_size:
                    goal_node.parent = new_node
                    tree.append(goal_node)
                    path_found = True
                    break
        
        if path_found:
            raw_path = []
            curr = goal_node
            while curr is not None:
                raw_path.append((curr.x, curr.y, curr.z))
                curr = curr.parent
            raw_path.reverse() 
            
            dense_path = []
            dense_step = 0.5 
            
            for i in range(len(raw_path) - 1):
                p1 = raw_path[i]
                p2 = raw_path[i+1]
                segment_dist = math.dist(p1, p2)
                
                num_inserts = max(1, int(segment_dist / dense_step))
                
                for j in range(num_inserts):
                    frac = j / num_inserts
                    ix = p1[0] + frac * (p2[0] - p1[0])
                    iy = p1[1] + frac * (p2[1] - p1[1])
                    iz = p1[2] + frac * (p2[2] - p1[2])
                    dense_path.append((ix, iy, iz))
            
            dense_path.append(raw_path[-1]) 
            self.waypoints = dense_path
            self.initialized = True
            self.get_logger().info(f"✅ RRT 规划成功！共生成 {len(self.waypoints)} 个高密度路点。")
        else:
            self.get_logger().error("❌ RRT 规划失败：未找到有效路径！")

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
                self.generate_rrt_path((curr_x, curr_y, curr_z))
                return
        except Exception:
            return

        if self.current_wp_idx >= len(self.waypoints):
            if not self.tasks_completed:
                self.get_logger().info("🎉 导航任务完成！AUV 终点悬停。")
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

        dist_xy = math.sqrt((curr_x - target[0])**2 + (curr_y - target[1])**2)
        dist_z = abs(curr_z - target[2])
        
        is_last_wp = (self.current_wp_idx == len(self.waypoints) - 1)
        xy_thresh = 0.5 if is_last_wp else 1.5
        z_thresh = 0.4 if is_last_wp else 1.0

        if dist_xy < xy_thresh and dist_z < z_thresh:
            self.get_logger().info(f"✅ 进度 [{self.current_wp_idx + 1}/{len(self.waypoints)}] | 坐标: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
            self.current_wp_idx += 1

    def draw_solid_box(self, ax, obs):
        x_min, x_max, y_min, y_max, z_min, z_max = obs
        v = [
            [x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]
        ]
        faces = [
            [v[0], v[1], v[5], v[4]], [v[1], v[2], v[6], v[5]], [v[2], v[3], v[7], v[6]], 
            [v[3], v[0], v[4], v[7]], [v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]]
        ]
        poly3d = Poly3DCollection(faces, facecolor='red', alpha=0.15, edgecolor='darkred', linewidths=1.0)
        ax.add_collection3d(poly3d)

    def draw_solid_sphere(self, ax, obs):
        """【新增】渲染 3D 完美球形曲面"""
        cx, cy, cz, r = obs
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 10)
        x = cx + r * np.outer(np.cos(u), np.sin(v))
        y = cy + r * np.outer(np.sin(u), np.sin(v))
        z = cz + r * np.outer(np.ones(np.size(u)), np.cos(v))
        # 绘制半透明球形网格面
        ax.plot_surface(x, y, z, color='orange', alpha=0.2, edgecolor='darkorange', linewidth=0.5)

    def save_mission_report(self):
        if not self.x_history: return
            
        print("\n📊 正在渲染复杂地形 RRT 避障分析图...")
        fig = plt.figure(figsize=(16, 7)) 
        
        wp_x = [wp[0] for wp in self.waypoints]
        wp_y = [wp[1] for wp in self.waypoints]
        wp_z = [wp[2] for wp in self.waypoints]

        # ================= 1. 左侧：3D 主视角 =================
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        ax1.plot(self.x_history, self.y_history, self.z_history, 'b-', label='AUV Track', alpha=0.8, linewidth=2)
        ax1.plot(wp_x, wp_y, wp_z, 'k--', label='RRT Path', alpha=0.5)
        
        # 渲染十字架矩形残骸
        for obs in self.box_obstacles:
            self.draw_solid_box(ax1, obs)
        # 渲染球形水雷
        for sphere in self.sphere_obstacles:
            self.draw_solid_sphere(ax1, sphere)
            
        ax1.plot([], [], 's', color='red', alpha=0.3, label='Box Obstacles')
        ax1.plot([], [], 'o', color='orange', alpha=0.3, label='Sphere Obstacles')
        if self.start_pos: ax1.scatter(*self.start_pos, color='green', marker='o', s=100, label='Start')
        ax1.scatter(*self.goal_pos, color='gold', marker='*', s=150, label='Goal')
        ax1.set_title('3D Overview', fontsize=14, weight='bold')
        ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Depth (m)')
        ax1.legend(loc='best')

        # ================= 2. 右侧：X-Y 俯视图 =================
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(self.x_history, self.y_history, 'b-', linewidth=2)
        ax2.plot(wp_x, wp_y, 'k--')
        
        # 渲染矩形 2D 投影
        for obs in self.box_obstacles:
            rect = patches.Rectangle((obs[0], obs[2]), obs[1]-obs[0], obs[3]-obs[2], color='red', alpha=0.3)
            ax2.add_patch(rect)
        # 渲染圆形 2D 投影
        for sphere in self.sphere_obstacles:
            circle = patches.Circle((sphere[0], sphere[1]), sphere[3], color='orange', alpha=0.3)
            ax2.add_patch(circle)
            
        if self.start_pos: ax2.scatter(self.start_pos[0], self.start_pos[1], color='green', marker='o', s=100)
        ax2.scatter(self.goal_pos[0], self.goal_pos[1], color='gold', marker='*', s=150)
        ax2.set_title('Top View (X-Y Plane)', fontsize=14, weight='bold')
        ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
        ax2.grid(True, linestyle='--', alpha=0.5)

        plt.suptitle('AUV Irregular & Spherical Obstacles Avoidance', fontsize=18, weight='bold')
        plt.tight_layout()
        
        # ================= 保存逻辑 =================
        save_dir = "/home/cjx/cjx_project/AUV/AUV_ws/log/实验数据记录"
        if not os.path.exists(save_dir): os.makedirs(save_dir)
            
        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M")
        full_path_png = os.path.join(save_dir, f"AUV_3D_RRT_Complex_{now_str}.png")
        full_path_pdf = os.path.join(save_dir, f"AUV_3D_RRT_Complex_{now_str}.pdf")
        
        plt.savefig(full_path_png, dpi=150)
        plt.savefig(full_path_pdf, dpi=150, format='pdf', bbox_inches='tight')
        print(f"💾 实验报告已成功双份保存:\n   📍 [PNG图]: {full_path_png}\n   📍 [PDF图]: {full_path_pdf}")
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