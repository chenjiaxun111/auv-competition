# Jetson 对 auvmaster 2.x 的适配说明

适配目标：`/home/cjx/cjx_project/auv比赛/auv-2026`

## RPC 对照

| Jetson动作 | auvmaster 2.x 方法 | 参数/结果 |
|---|---|---|
| 读取状态并续心跳 | `status` | 返回 `temperature_celsius`、`heading_degrees`、`depth_meters` |
| 读取传感器有效性 | `pid_runtime_status` | `depth_position.sensor_valid`、`yaw_angle.sensor_valid` |
| 四轴运动 | `move` | `x=sway`、`y=surge`、`z=heave`、`yaw=yaw`，范围 `[-1,1]` |
| 相对深度/航向目标 | `auv_target` | `relative_depth_meters`、`relative_heading_degrees` |
| 六路计算输出 | `propeller_power` | 六路相对 1500 us 的有符号偏移，不是硬件回读 |

有效的 `move` 和 `auv_target` 返回 JSON `null`。这代表服务器已接受并执行处理函数，但新协议没有命令序号或独立 ACK 对象。

## 坐标约定

ROS 上层继续使用：

- 深度向下为正，单位米。
- `Wrench.force.x` 为 surge。
- `Wrench.force.y` 为 sway。
- `Wrench.force.z` 为 heave。
- `Wrench.torque.z` 为 yaw。

当前 auvmaster 2.x 通过 `AUV_DEPTH_FEEDBACK_DIRECTION=-1` 使用向下为负的深度，且绝对目标范围为 `[-2,0] m`。桥接节点通过 `sensor_sign.depth=-1.0` 双向转换，因此比赛控制器仍可使用 `0.15～1.10 m` 的正深度。

## 安全与运行约束

- Jetson 命令超过 0.30 秒未更新时，桥接节点持续发送零运动。
- 新固件只有 `status` 会更新心跳，`move` 本身不会更新；桥接节点和测试工具已定期发送 `status`。
- 新固件的 `AUV_HEARTBEAT_TIMEOUT_MS` 当前为 3000 ms。网络完全中断时，底层最迟约 3 秒后停止；是否缩短由 NanoPi 团队决定。
- AUV 模式下，只要传感器有效且手动轴为零，底层会自动进入深度/航向保持；新协议没有远程 PID 启停开关。
- `onboard_depth_hold_enabled` 当前只控制 Jetson 是否允许发送深度目标，不会关闭底层自动保持。

## 验证结果

- 本机纯协议与逻辑测试：17 项通过（L1）。
- Jetson ROS 2 Humble 构建及完整包测试：28 项通过（L2）。
- 新版已部署到 NanoPi 的 `/home/pi/auvmaster-2`，NanoPi 原生 ARM 编译和启动通过。
- 实机 PCA9685 位于 `/dev/i2c-1`、地址 `0x40`；配置修正后硬件初始化成功，RPC 正常监听端口 8888。
- Jetson 到 NanoPi 的全零实机协议验证通过（L4）：分片 HTTP、`status`、`pid_runtime_status`、全零 `move`、六路 `propeller_power` 和最终停车均通过。
- 未安装深度计和陀螺仪时，底层正确返回 `depth_valid=false`、`yaw_valid=false`；本次没有执行非零运动测试。
- 验收后 NanoPi 进程已正常停止，未设置开机自启。

## 测试命令

无运动协议检查：

```bash
cd /home/auv/ros2_ws/src/rov_thruster_bridge
python3 firmware_protocol_test.py --host 192.168.137.219
```

短时非零台架检查必须明确授权：

```bash
python3 firmware_protocol_test.py \
  --host 192.168.137.219 \
  --allow-motion
```

无桨四轴正反向混控检查：

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
ros2 run rov_thruster_bridge axis_sweep_test --allow-motion
```

脚本默认测试四轴正负方向、回读六路混控值，并在每阶段和退出时验证全零。
可用 `--axes surge yaw` 只测试指定轴；`--amplitude` 被限制在 `0.25～0.50`。
这些结果属于无桨硬件通电 L4 验证，不是带桨运动或电气波形验证。

ROS桥接启动：

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
ros2 launch rov_thruster_bridge thruster_bridge.launch.py
```

比赛频率全零通信压测：

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
ros2 run rov_thruster_bridge communication_stress_test
```

默认持续 600 秒，以 20 Hz 发送全零运动命令，同时各以 10 Hz 查询状态和 PID 传感器有效性，共 40 RPC/s。可用 `--duration 1800` 执行 30 分钟浸泡测试。脚本不会发送非零运动量，结束时连续回零并检查六路 `propeller_power`；JSON 报告保存在 `/home/auv/ros2_ws/stress_reports/`。

2026-09-14 的 10 秒实机回归为 400/400 请求成功、无漏周期，控制/遥测实际频率分别为 20.00/10.00 Hz，最终六路软件输出为零（L4）。这不是 PCA9685 寄存器或 PWM 电气波形回读。
