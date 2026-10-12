# nokov_bridge — Nokov/XINGYING 动捕 TF 桥接包

连接 XINGYING 动捕系统，将 Nokov 刚体位姿实时发布为 ROS 2 TF 变换链，供 `auv_control` 规划器和 PID 追踪器使用。

---

## 整体架构

```
XINGYING 动捕 (10.1.1.198:3883)
      │ VRPN (TCP)
      ▼
┌─────────────────┐
│  vrpn_mocap      │  ← ros-humble-vrpn-mocap (apt 安装)
│  client_node     │     订阅 VRPN 数据, 发布 PoseStamped
└────────┬────────┘
         │ /vrpn_mocap/{tracker}/pose0 (PoseStamped, 150Hz, mm, BEST_EFFORT)
         ▼
┌─────────────────┐
│  nokov_tf_bridge │  ← 本包核心节点
│                  │     mm→m, 坐标系修正, 标定偏移, TF 广播
└────────┬────────┘
         │ TF: map → odom → base_link
         │ /auv/pose, /auv/odom
         ▼
┌─────────────────┐      ┌──────────────────┐
│  hybrid_planner  │ ──▶ │  auv_pid_tracker  │  ← auv_control 包
│  (auv_control)   │      │  (auv_control)    │
└──────────────────┘      └────────┬─────────┘
                                   │ /auv/control_command (Wrench)
                                   ▼
                          ┌─────────────────┐
                          │ thruster_bridge  │  ← rov_thruster_bridge 包
                          │                  │     Wrench → 6ch PWM → JSON-RPC
                          └────────┬────────┘
                                   │ HTTP (192.168.137.219:8888)
                                   ▼
                          ┌─────────────────┐
                          │   ROV 硬件       │  6 台推进器
                          └─────────────────┘
```

---

## 文件结构

```
nokov_bridge/
├── package.xml
├── setup.py
├── README.md
├── launch/
│   └── nokov_bridge.launch.py       # vrpn_mocap + nokov_tf_bridge 一键启动
├── config/
│   └── nokov_params.yaml            # 所有参数 (tracker名/坐标系/标定偏移)
├── nokov_bridge/
│   ├── __init__.py
│   ├── nokov_tf_bridge.py           # 核心: PoseStamped → TF 桥接
│   └── diagnose_nokov.py            # ROS2 话题诊断工具
└── scripts/
    └── vrpn_raw_diagnose.py         # VRPN 原始 TCP 诊断 (无需 ROS)
```

---

## 水池标定

每次重新架设动捕（相机位置变化），需要重新标定，将 map 原点对齐到水池物理参考点。

### 原理

```
公式: map坐标 = Nokov坐标 + offset
                     ↑
              offset = - (Nokov在原点的读数)
```

### 标定步骤

**1. 暂时清零 offset**

编辑 [nokov_params.yaml](config/nokov_params.yaml)：

```yaml
offset_x: 0.0
offset_y: 0.0
offset_z: 0.0
```

**2. 启动动捕链路**

```bash
ros2 launch nokov_bridge nokov_bridge.launch.py server:=10.1.1.198
```

**3. 把 ROV 放到水池原点**

将粘贴了反光球的刚体放在你想定义为 `(0, 0, 0)` 的物理参考点（如水池西南角底部）。

**4. 读取 Nokov 原始坐标**

```bash
ros2 run tf2_ros tf2_echo map base_link -r 1
```

记下 Translation 数值，例如：

```
Translation: [2.500, -1.300, 1.800]
```

**5. 计算 offset 并填入 yaml**

```
offset_x = -2.500   (取反)
offset_y =  1.300   (取反, 注意符号)
offset_z = -1.800   (取反)
```

编辑 [nokov_params.yaml](config/nokov_params.yaml)：

```yaml
offset_x: -2.500
offset_y: 1.300
offset_z: -1.800
```

**6. 重启并验证**

```bash
ros2 launch nokov_bridge nokov_bridge.launch.py server:=10.1.1.198
ros2 run tf2_ros tf2_echo map base_link -r 1
# 应看到 Translation: [0, 0, 0] 附近
```

**7. 距离验证**

把 ROV 移到水池另一已知位置，检查两点间距离是否与卷尺测量一致：

```
距离 = √((x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²)
```

---

## nokov_tf_bridge 节点

**源码:** [nokov_tf_bridge.py](nokov_bridge/nokov_tf_bridge.py)

| 处理步骤 | 说明 |
|----------|------|
| 单位转换 | Nokov 输出 mm，× `scale` (0.001) 转 m |
| 坐标系修正 | 可选 Z 翻转 / Y 翻转 / XY 交换 |
| 标定偏移 | 叠加 offset_* 偏移，对齐水池原点 |
| TF 广播 | `map → odom` (带偏移的 Nokov 数据) + `odom → base_link` (identity) |
| 辅助话题 | `/auv/pose` (PoseStamped), `/auv/odom` (Odometry) |

### 参数 (nokov_params.yaml)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tracker_name` | `ROV` | XINGYING 中刚体名称 |
| `sensor_id` | `0` | 传感器编号 |
| `vrpn_namespace` | `/vrpn_mocap` | vrpn_mocap 命名空间 |
| `scale` | `0.001` | mm → m 缩放 |
| `invert_z` | `false` | Z 轴翻转 (Nokov Z-up → 保持一致) |
| `invert_y` | `false` | Y 轴翻转 |
| `swap_xy` | `false` | XY 轴交换 |
| `offset_x/y/z` | `0.0` | 标定平移偏移 (m) |
| `offset_roll/pitch/yaw` | `0.0` | 标定旋转偏移 (rad) |

---

## 启动方式

### 仅动捕链路

```bash
ros2 launch nokov_bridge nokov_bridge.launch.py server:=10.1.1.198
```

### 全链路 (动捕 + 规划器 + PID + ROV 推进器)

```bash
ros2 launch auv_bringup pool_mission.launch.py server:=10.1.1.198
```

### 查看实时位姿

```bash
ros2 run tf2_ros tf2_echo map base_link -r 10
# 或
ros2 topic echo /auv/pose
```

### 启动预检清单

| 检查项 | 说明 |
|--------|------|
| ✅ XINGYING 动捕已开启 | 刚体 "ROV" 被追踪, VRPN 广播已启用 |
| ✅ `ros-humble-vrpn-mocap` 已安装 | `sudo apt install ros-humble-vrpn-mocap` |
| ✅ 能 ping 通动捕服务器 | `ping 10.1.1.198` |
| ✅ 标定已完成 | offset_* 已填入水池原点值 |
| ✅ `auv_control` 包已编译 | 提供 hybrid_planner + auv_pid_tracker |

---

## 诊断工具

### diagnose_nokov.py — ROS 2 话题诊断

动捕链路就绪后，自动发现并订阅动捕相关话题，分析数据格式。

```bash
ros2 run nokov_bridge diagnose_nokov
```

### vrpn_raw_diagnose.py — VRPN 原始诊断 (无需 ROS)

直接用 TCP 连接 VRPN 端口，不依赖 ROS 2。用于排查 VRPN 连接问题。

```bash
python3 scripts/vrpn_raw_diagnose.py --host 10.1.1.198 --duration 15
```

---

## Topic & TF 速查

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/vrpn_mocap/ROV/pose0` | PoseStamped | vrpn_mocap → bridge | Nokov 原始数据 (mm, 150Hz) |
| `map → odom` | TF | bridge → 全局 | 偏移后的 Nokov 位姿 |
| `odom → base_link` | TF | bridge → 全局 | identity |
| `/auv/pose` | PoseStamped | bridge → 全局 | 转换后的位姿 (m) |
| `/auv/odom` | Odometry | bridge → 全局 | 里程计格式 |
| `map → dummy_target` | TF | planner → PID | 当前目标航点 |
| `/auv/control_command` | Wrench | PID → thruster | 三轴力/力矩指令 |

---

## 常见问题

### Q: 动捕没有数据？
- 确认 XINGYING 已启动且刚体 "ROV" 被追踪
- 检查 VRPN 广播端口: `timeout 3 bash -c "echo > /dev/tcp/10.1.1.198/3883" && echo OK`
- 用 `ros2 topic echo /vrpn_mocap/ROV/pose0` 验证原始数据

### Q: 重启后坐标又不对了？
- 重新走一遍标定流程：清 offset → 放原点 → 读坐标 → 取反填入 → 重启

### Q: Z 轴方向反了？
- 调 `invert_z` 参数: `ros2 launch nokov_bridge nokov_bridge.launch.py invert_z:=true`

### Q: QoS 不匹配 (RELIABILITY_QOS_POLICY warning)？
- `ros2 topic echo` 使用不兼容的 RELIABLE QoS, 属于正常现象
- nokov_tf_bridge 已使用 BEST_EFFORT, 不会丢数据
