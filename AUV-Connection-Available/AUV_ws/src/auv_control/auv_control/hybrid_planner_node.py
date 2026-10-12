#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from std_msgs.msg import String  
import json                      

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.patches as patches                  
import numpy as np 
import math
import random
import os
from datetime import datetime
import time
import sys

# 引入 B 样条平滑数学库
import scipy.interpolate as si

class RRTNode:
    def __init__(self, x, y, z):
        self.x = x; self.y = y; self.z = z; self.parent = None

class HybridPlannerNode(Node):
    def __init__(self):
        super().__init__('hybrid_planner_node')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.start_pos = None     
        self.initialized = False  
        self.current_wp_idx = 0
        self.tasks_completed = False
        self.x_history, self.y_history, self.z_history = [], [], []

        self.goal_pos = (75.0, -10.0, -25.0) 
        self.bounds = {"x": (15, 85), "y": (-50, 0), "z": (-35, -4.0)} 
        
        # 1. 复杂静态环境
        self.box_obstacles = [
            (50.0, 53.0, -35.0, -15.0, -11.0, -5.0),  
            (44.0, 50.0, -18.0, -15.0, -11.0, -5.0),  
            (30.0, 45.0, -35.0, -30.0, -25.0, -20.0), 
            (35.0, 40.0, -38.0, -27.0, -28.0, -15.0), 
        ]
        self.sphere_obstacles = [(35.0, -10.0, -20.0, 4.0), (65.0, -35.0, -15.0, 3.5)]
        
        # 2. 动态入侵 AUV
        self.dyn_obs = {
            'pos': [45.0, -25.0, -18.0], 'target': [45.0, -25.0, -18.0], 
            'bounds': {'x': (35.0, 55.0), 'y': (-40.0, -10.0), 'z': (-25.0, -10.0)}, 
            'speed': 2.0, 'radius': 2.5, 'influence': 12.0    
        }
        self.dyn_obs_history_x, self.dyn_obs_history_y, self.dyn_obs_history_z = [], [], []
        self.last_time = time.time()
        
        # 3. HoloOcean & UI
        self.obs_pub = self.create_publisher(String, '/holoocean_obstacles', 10)
        self.obs_pub_timer = self.create_timer(2.0, self.publish_obstacles)
        self.current_elastic_band = ([], []) # 用于在雷达上绘制动态 B 样条弹力带

        self.loop_count = 0
        plt.ion() 
        self.rt_fig = None; self.rt_ax = None
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("🚀 B 样条动力学平滑引擎 & 实时雷达监控已启动...")

    def publish_obstacles(self):
        msg = String()
        msg.data = json.dumps({"boxes": self.box_obstacles, "spheres": self.sphere_obstacles})
        self.obs_pub.publish(msg)

    def check_collision(self, p1, p2):
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            cx, cy, cz = p1[0] + t*(p2[0]-p1[0]), p1[1] + t*(p2[1]-p1[1]), p1[2] + t*(p2[2]-p1[2])
            for obs in self.box_obstacles:
                if obs[0]<=cx<=obs[1] and obs[2]<=cy<=obs[3] and obs[4]<=cz<=obs[5]: return True 
            for sphere in self.sphere_obstacles:
                if math.dist((cx, cy, cz), (sphere[0], sphere[1], sphere[2])) <= sphere[3]: return True
        return False

    def get_clearance(self, pos):
        min_dist = float('inf')
        cx, cy, cz = pos
        for obs in self.box_obstacles:
            d = math.dist((cx, cy, cz), (max(obs[0], min(cx, obs[1])), max(obs[2], min(cy, obs[3])), max(obs[4], min(cz, obs[5]))))
            if d < min_dist: min_dist = d
        for sphere in self.sphere_obstacles:
            d = math.dist((cx, cy, cz), (sphere[0], sphere[1], sphere[2])) - sphere[3]
            if d < min_dist: min_dist = max(0.0, d)
        return min_dist

    def check_realtime_collision(self, curr_pos):
        cx, cy, cz = curr_pos
        for obs in self.box_obstacles:
            if (obs[0]-0.2)<=cx<=(obs[1]+0.2) and (obs[2]-0.2)<=cy<=(obs[3]+0.2) and (obs[4]-0.2)<=cz<=(obs[5]+0.2): return "STATIC_TERRAIN"
        for sphere in self.sphere_obstacles:
            if math.dist((cx, cy, cz), (sphere[0], sphere[1], sphere[2])) <= sphere[3]: return "STATIC_REEF"
        if math.dist((cx, cy, cz), tuple(self.dyn_obs['pos'])) <= self.dyn_obs['radius']: return "INTRUDER_AUV"
        return None

    def setup_realtime_plot(self):
        self.rt_fig, self.rt_ax = plt.subplots(figsize=(8, 6))
        self.rt_fig.canvas.manager.set_window_title('Live Radar: B-Spline Elastic Band Evasion')
        
        wp_x, wp_y = [wp[0] for wp in self.waypoints], [wp[1] for wp in self.waypoints]
        self.rt_ax.plot(wp_x, wp_y, 'k--', alpha=0.3, label='Global B-Spline Path')
        
        for obs in self.box_obstacles: self.rt_ax.add_patch(patches.Rectangle((obs[0], obs[2]), obs[1]-obs[0], obs[3]-obs[2], color='gray', alpha=0.4))
        for sphere in self.sphere_obstacles: self.rt_ax.add_patch(patches.Circle((sphere[0], sphere[1]), sphere[3], color='orange', alpha=0.3))
            
        b = self.dyn_obs['bounds']
        self.rt_ax.add_patch(patches.Rectangle((b['x'][0], b['y'][0]), b['x'][1]-b['x'][0], b['y'][1]-b['y'][0], fill=False, edgecolor='red', linestyle=':', alpha=0.5))

        if self.start_pos: self.rt_ax.scatter(self.start_pos[0], self.start_pos[1], color='green', marker='o', s=100)
        self.rt_ax.scatter(self.goal_pos[0], self.goal_pos[1], color='gold', marker='*', s=150)

        # 【新增】：局部的 B 样条动态弹力带 (青色粗线)
        self.rt_elastic_band_line, = self.rt_ax.plot([], [], 'c-', linewidth=3, alpha=0.7, label='Dynamic Elastic Band')
        
        self.rt_auv_line, = self.rt_ax.plot([], [], 'b-', linewidth=2, label='AUV True Track')
        self.rt_auv_marker, = self.rt_ax.plot([], [], 'bo', markersize=6) 
        self.rt_dyn_mine_line, = self.rt_ax.plot([], [], 'r-', alpha=0.6, label='Intruder AUV')
        self.rt_dyn_mine_marker, = self.rt_ax.plot([], [], 'ro', markersize=10, alpha=0.5) 
        
        self.rt_ax.set_xlim(self.bounds["x"][0]-5, self.bounds["x"][1]+5); self.rt_ax.set_ylim(self.bounds["y"][0]-5, self.bounds["y"][1]+5)
        self.rt_ax.set_title("Live Radar: Receding Horizon Elastic Band", weight='bold', color='darkblue')
        self.rt_ax.legend(loc='best'); self.rt_ax.grid(True, linestyle='--', alpha=0.5)
        self.rt_fig.canvas.draw(); self.rt_fig.canvas.flush_events()

    def update_realtime_plot(self):
        if not self.rt_fig: return
        try:
            self.rt_auv_line.set_data(self.x_history, self.y_history)
            if self.x_history: self.rt_auv_marker.set_data([self.x_history[-1]], [self.y_history[-1]])
            self.rt_dyn_mine_line.set_data(self.dyn_obs_history_x, self.dyn_obs_history_y)
            if self.dyn_obs_history_x: self.rt_dyn_mine_marker.set_data([self.dyn_obs_history_x[-1]], [self.dyn_obs_history_y[-1]])
            
            # 更新弹力带显示
            if self.current_elastic_band[0]:
                self.rt_elastic_band_line.set_data(self.current_elastic_band[0], self.current_elastic_band[1])
                
            self.rt_fig.canvas.draw_idle(); self.rt_fig.canvas.flush_events()
        except Exception: pass

    def generate_rrt_path(self, start_pos):
        self.get_logger().info("🌳 执行 1级规划：RRT 空间探路与自适应步长...")
        self.start_pos = start_pos
        step_size, max_iter, path_found = 2.0, 5000, False  
        tree = [RRTNode(*start_pos)]; goal_node = RRTNode(*self.goal_pos)
        
        for _ in range(max_iter):
            rand_pos = self.goal_pos if random.random() < 0.1 else (random.uniform(*self.bounds["x"]), random.uniform(*self.bounds["y"]), random.uniform(*self.bounds["z"]))
            nearest_node = min(tree, key=lambda n: math.dist((n.x, n.y, n.z), rand_pos))
            d = math.dist((nearest_node.x, nearest_node.y, nearest_node.z), rand_pos)
            if d == 0: continue
            
            new_pos = (nearest_node.x + min(step_size, d) * (rand_pos[0]-nearest_node.x)/d, nearest_node.y + min(step_size, d) * (rand_pos[1]-nearest_node.y)/d, nearest_node.z + min(step_size, d) * (rand_pos[2]-nearest_node.z)/d)
            if not self.check_collision((nearest_node.x, nearest_node.y, nearest_node.z), new_pos):
                new_node = RRTNode(*new_pos); new_node.parent = nearest_node; tree.append(new_node)
                if math.dist((new_node.x, new_node.y, new_node.z), self.goal_pos) < step_size:
                    goal_node.parent = new_node; tree.append(goal_node); path_found = True; break
        
        if path_found:
            raw_path, curr = [], goal_node
            while curr: raw_path.append((curr.x, curr.y, curr.z)); curr = curr.parent
            raw_path.reverse() 
            
            dense_path = []
            for i in range(len(raw_path)-1):
                p1, p2 = raw_path[i], raw_path[i+1]
                mid_p = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2)
                clearance = self.get_clearance(mid_p)
                
                if clearance < 3.0: dynamic_step = 0.2
                elif clearance > 8.0: dynamic_step = 1.5
                else: dynamic_step = 0.2 + (clearance - 3.0)/5.0 * 1.3
                
                num_inserts = max(1, int(math.dist(p1, p2)/dynamic_step))
                for j in range(num_inserts):
                    frac = j / num_inserts
                    dense_path.append((p1[0] + frac*(p2[0]-p1[0]), p1[1] + frac*(p2[1]-p1[1]), p1[2] + frac*(p2[2]-p1[2])))
            dense_path.append(raw_path[-1]) 

            # ================= 1级平滑：全局 B 样条把 RRT 锯齿全部拉直 =================
            self.get_logger().info("🌊 执行 2级规划：全局 B-Spline C2 平滑处理...")
            unique_dense = []
            for p in dense_path:
                if not unique_dense or math.dist(p, unique_dense[-1]) > 0.05: unique_dense.append(p)
                
            if len(unique_dense) >= 4:
                pts = np.array(unique_dense).T
                tck, _ = si.splprep(pts, s=2.0, k=3)
                smooth_pts = si.splev(np.linspace(0, 1, len(unique_dense)), tck)
                self.waypoints = list(zip(smooth_pts[0], smooth_pts[1], smooth_pts[2]))
            else:
                self.waypoints = unique_dense
            # =========================================================================

            self.initialized = True
            self.setup_realtime_plot()
            self.get_logger().info(f"✅ 基准路径就绪！启动 APF 动态弹力带循环。")
        else: self.get_logger().error(f"❌ 探路失败，请重试！")

    def loop(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            curr_pos = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            self.x_history.append(curr_pos[0]); self.y_history.append(curr_pos[1]); self.z_history.append(curr_pos[2])
            if not self.initialized: self.generate_rrt_path(curr_pos); return
        except Exception: return

        if self.current_wp_idx >= len(self.waypoints):
            if not self.tasks_completed: self.get_logger().info("🎉 任务圆满完成！"); self.tasks_completed = True
            return

        collision_target = self.check_realtime_collision(curr_pos)
        if collision_target:
            self.get_logger().fatal(f"💥 发生碰撞 [{collision_target}]！触发 E-Stop！")
            self.timer.cancel(); self.save_mission_report(); sys.exit(1)

        # ================= 入侵 AUV 更新 =================
        dt = time.time() - getattr(self, 'last_time', time.time()); self.last_time = time.time()
        dyn_pos, dyn_target = self.dyn_obs['pos'], self.dyn_obs['target']
        if math.dist(dyn_pos, dyn_target) < 0.5:
            self.dyn_obs['target'] = [random.uniform(*self.dyn_obs['bounds']['x']), random.uniform(*self.dyn_obs['bounds']['y']), random.uniform(*self.dyn_obs['bounds']['z'])]
        else:
            dir_vec = [(dyn_target[i]-dyn_pos[i])/math.dist(dyn_pos, dyn_target) for i in range(3)]
            for i in range(3): self.dyn_obs['pos'][i] += dir_vec[i] * min(self.dyn_obs['speed']*dt, math.dist(dyn_pos, dyn_target))
        dyn_pos_tuple = tuple(self.dyn_obs['pos'])
        self.dyn_obs_history_x.append(dyn_pos_tuple[0]); self.dyn_obs_history_y.append(dyn_pos_tuple[1]); self.dyn_obs_history_z.append(dyn_pos_tuple[2])

        # ================= 🚀 核心：3级规划 - 滚动时域 B 样条弹力带 (Elastic Band) =================
        # 提取前方 8 个航点构成的“局部视野”
        lookahead_idx = min(self.current_wp_idx + 8, len(self.waypoints))
        lookahead_wps = self.waypoints[self.current_wp_idx : lookahead_idx]
        
        # 为了保证平滑曲线从 AUV 车头自然延展，必须把当前真实位置加进去
        local_pts = [list(curr_pos)]
        apf_active = False
        
        # 将 APF 斥力叠加到未来的几个点上 (而不是只推当前点)
        for wp in lookahead_wps:
            p = list(wp)
            dist_to_dyn = math.dist(p, dyn_pos_tuple)
            if dist_to_dyn < self.dyn_obs['influence']:
                apf_active = True
                dir_v = [p[i]-dyn_pos_tuple[i] for i in range(3)]
                norm = math.sqrt(sum(d**2 for d in dir_v))
                if norm > 0: dir_v = [d/norm for d in dir_v]
                rep_mag = min(15.0 * (1.0/dist_to_dyn - 1.0/self.dyn_obs['influence']), 8.0) # 限制斥力上限防拉断
                for i in range(3): p[i] += dir_v[i] * rep_mag
            local_pts.append(p)

        # 剔除重合点以防 B 样条矩阵计算奇异崩溃
        unique_local = []
        for p in local_pts:
            if not unique_local or math.dist(p, unique_local[-1]) > 0.05: unique_local.append(p)

        # 如果受到 APF 挤压，执行局部弹力带平滑打磨
        if apf_active and len(unique_local) >= 4:
            try:
                pts_arr = np.array(unique_local).T
                tck, _ = si.splprep(pts_arr, s=3.0, k=3) # 三次样条高度平滑
                
                # 在雷达上画出这条青色的预判弹力带 (0 到 1 全路段)
                band_pts = si.splev(np.linspace(0, 1, 30), tck)
                self.current_elastic_band = (list(band_pts[0]), list(band_pts[1]))
                
                # 提取曲线上 15% 处作为 AUV 底层 PID 的当前平滑引导点
                smooth_target = si.splev(0.15, tck)
                final_target = [float(smooth_target[0]), float(smooth_target[1]), float(smooth_target[2])]
                self.get_logger().warn(f'🛡️ 弹力带介入：成功将 APF 锐角打磨成连续圆弧！')
            except Exception:
                final_target = unique_local[1] if len(unique_local)>1 else list(self.goal_pos)
        else:
            final_target = list(self.waypoints[self.current_wp_idx])
            self.current_elastic_band = ([], []) # 关闭弹力带显示
        # =========================================================================================

        # 发送目标点给底层 TF
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg(); t_msg.header.frame_id = 'map'; t_msg.child_frame_id = 'dummy_target'
        t_msg.transform.translation.x, t_msg.transform.translation.y, t_msg.transform.translation.z = final_target
        t_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_msg)

        is_last = (self.current_wp_idx == len(self.waypoints) - 1)
        if math.dist(curr_pos[:2], self.waypoints[self.current_wp_idx][:2]) < (0.5 if is_last else 1.5) and abs(curr_pos[2] - self.waypoints[self.current_wp_idx][2]) < (0.4 if is_last else 1.0):
            self.current_wp_idx += 1

        self.loop_count += 1
        if self.loop_count % 3 == 0: self.update_realtime_plot()

    # ... [下方的绘图代码 draw_solid_box, draw_solid_sphere, save_mission_report 完全保持原样，无需修改] ...
    def draw_solid_box(self, ax, obs):
        x_min, x_max, y_min, y_max, z_min, z_max = obs
        v = [[x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min], [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]]
        faces = [[v[0], v[1], v[5], v[4]], [v[1], v[2], v[6], v[5]], [v[2], v[3], v[7], v[6]], [v[3], v[0], v[4], v[7]], [v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]]]
        ax.add_collection3d(Poly3DCollection(faces, facecolor='gray', alpha=0.25, edgecolor='black', linewidths=0.5))

    def draw_solid_sphere(self, ax, obs, color='orange', alpha=0.2):
        cx, cy, cz, r = obs
        u, v = np.linspace(0, 2 * np.pi, 20), np.linspace(0, np.pi, 10)
        x = cx + r * np.outer(np.cos(u), np.sin(v))
        y = cy + r * np.outer(np.sin(u), np.sin(v))
        z = cz + r * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color=color, alpha=alpha, edgecolor=color, linewidth=0.2)

    def save_mission_report(self):
        if not self.x_history: return
        plt.ioff(); plt.close(self.rt_fig) if self.rt_fig else None
            
        print("\n📊 正在渲染 [B 样条平滑自适应 3D 避障] 严谨论证报告...")
        fig = plt.figure(figsize=(20, 14)) 
        wp_x, wp_y, wp_z = [wp[0] for wp in self.waypoints], [wp[1] for wp in self.waypoints], [wp[2] for wp in self.waypoints]

        def plot_2d_proj(ax, view_x, view_y, wp_vx, wp_vy, idx1, idx2):
            ax.plot(view_x, view_y, 'b-', linewidth=2); ax.plot(wp_vx, wp_vy, 'k--', alpha=0.3)
            for obs in self.box_obstacles: ax.add_patch(patches.Rectangle((obs[idx1], obs[idx2]), obs[idx1+1]-obs[idx1], obs[idx2+1]-obs[idx2], color='gray', alpha=0.4))
            for sph in self.sphere_obstacles: ax.add_patch(patches.Circle((sph[idx1//2], sph[idx2//2]), sph[3], color='orange', alpha=0.3))
            if self.dyn_obs_history_x:
                dyn_h = [self.dyn_obs_history_x, self.dyn_obs_history_y, self.dyn_obs_history_z]
                ax.plot(dyn_h[idx1//2], dyn_h[idx2//2], 'r-', alpha=0.6)
                ax.add_patch(patches.Circle((dyn_h[idx1//2][-1], dyn_h[idx2//2][-1]), self.dyn_obs['radius'], color='red', alpha=0.4))
            if self.start_pos: ax.scatter(self.start_pos[idx1//2], self.start_pos[idx2//2], color='green', marker='o', s=100)
            ax.scatter(self.goal_pos[idx1//2], self.goal_pos[idx2//2], color='gold', marker='*', s=150)
            ax.grid(True, linestyle='--', alpha=0.5)

        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(self.x_history, self.y_history, self.z_history, 'b-', label='Smooth Elastic Band Track', alpha=0.8, linewidth=2)
        ax1.plot(wp_x, wp_y, wp_z, 'k--', label='Global B-Spline Path', alpha=0.3) 
        for obs in self.box_obstacles: self.draw_solid_box(ax1, obs)
        for sphere in self.sphere_obstacles: self.draw_solid_sphere(ax1, sphere)
        if self.dyn_obs_history_x:
            ax1.plot(self.dyn_obs_history_x, self.dyn_obs_history_y, self.dyn_obs_history_z, 'r-', alpha=0.6, label='Intruder AUV')
            self.draw_solid_sphere(ax1, (self.dyn_obs_history_x[-1], self.dyn_obs_history_y[-1], self.dyn_obs_history_z[-1], self.dyn_obs['radius']), color='red', alpha=0.4)
        if self.start_pos: ax1.scatter(*self.start_pos, color='green', marker='o', s=100, label='Start')
        ax1.scatter(*self.goal_pos, color='gold', marker='*', s=150, label='Goal')
        ax1.set_title('3D Kinodynamic Evasion', fontsize=14, weight='bold'); ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z'); ax1.legend(loc='best')

        ax2 = fig.add_subplot(2, 2, 2); plot_2d_proj(ax2, self.x_history, self.y_history, wp_x, wp_y, 0, 2); ax2.set_title('Top View', fontsize=14, weight='bold')
        ax3 = fig.add_subplot(2, 2, 3); plot_2d_proj(ax3, self.x_history, self.z_history, wp_x, wp_z, 0, 4); ax3.set_title('Side View', fontsize=14, weight='bold')
        ax4 = fig.add_subplot(2, 2, 4); plot_2d_proj(ax4, self.y_history, self.z_history, wp_y, wp_z, 2, 4); ax4.set_title('Front View', fontsize=14, weight='bold')

        plt.suptitle('Multi-AUV Kinodynamic Collision Avoidance (B-Spline Elastic Band)', fontsize=18, weight='bold'); plt.tight_layout()
        save_dir = "/home/cjx/cjx_project/AUV/AUV_ws/log/实验数据记录"
        if not os.path.exists(save_dir): os.makedirs(save_dir)
        now_str = datetime.now().strftime("%Y-%m-%d_%H-%M")
        plt.savefig(os.path.join(save_dir, f"AUV_BSpline_{now_str}.pdf"), dpi=150, format='pdf', bbox_inches='tight')
        plt.show() 

def main(args=None):
    rclpy.init(args=args)
    node = HybridPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.save_mission_report()
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()