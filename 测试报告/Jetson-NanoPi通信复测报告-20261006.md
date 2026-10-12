# Jetson—NanoPi 通信复测报告（2026-10-06）

## 结论

Jetson 与 NanoPi 的网络和 JSON-RPC 应用层通信均可用。30 秒比赛频率全零压测通过，共完成 1200 次 RPC 请求，失败 0 次、漏周期 0 次，最终六路推进器软件目标值全部为 0。

本次仅发送 `x=0、y=0、z=0、yaw=0`，未发送任何非零运动指令。

## 测试环境

- Jetson 有线地址：`192.168.137.1/24`
- NanoPi 地址：`192.168.137.219/24`
- NanoPi JSON-RPC：`192.168.137.219:8888`
- NanoPi 程序：`/home/pi/auv-2026/build/auvmaster`
- Jetson 工作空间：`/home/auv/ros2_ws`
- 测试程序：`ros2 run rov_thruster_bridge communication_stress_test`
- 原始 JSON：`/home/auv/ros2_ws/stress_reports/jetson-nanopi-20261006-30s.json`

## 网络结果

- Jetson 到 NanoPi：5/5 响应，丢包率 0%
- 往返延迟：平均约 0.42 ms，最大约 0.49 ms
- 测试前后端口 `8888` 均保持监听
- 压测结束后 `auvmaster` 进程仍正常运行

## 应用层压测结果

| 项目 | 结果 |
|---|---:|
| 测试时长 | 30.00 s |
| RPC 总数 | 1200 |
| `move` 成功 | 600/600 |
| `status` 成功 | 300/300 |
| `pid_runtime_status` 成功 | 300/300 |
| 失败请求 | 0 |
| 最大连续失败 | 0 |
| 运动发送频率 | 20.00 Hz |
| 遥测查询频率 | 10.00 Hz |
| 运动漏周期 | 0 |
| 遥测漏周期 | 0 |
| 最终全零验证 | 通过 |

### RPC 延迟

| 方法 | 平均 | P95 | P99 | 最大 |
|---|---:|---:|---:|---:|
| `move` | 1.39 ms | 1.81 ms | 2.01 ms | 2.41 ms |
| `status` | 1.61 ms | 2.00 ms | 2.34 ms | 3.64 ms |
| `pid_runtime_status` | 1.58 ms | 1.75 ms | 1.83 ms | 2.14 ms |

`move` 调度最大迟到约 5.11 ms，远小于 50 ms 控制周期；测试未跳过任何调度槽。

## 最终状态回读

六路 `propeller_power`：

```text
left_up=0
right_up=0
left_middle=0
right_middle=0
left_down=0
right_down=0
```

遥测状态：

- `yaw_valid=true`，航向数据可读；
- `depth_valid=false`，与当前深度传感器不可用的实际状态一致；
- 深度、航向和横滚闭环均为 `active=false`；
- 全部 PID 输出为 0。

## 验证边界

本次达到 L4：真实 Jetson、NanoPi 和底层程序已上电，完成真实网络和协议通信，但只发送全零运动指令。

测试证明 Jetson 到 NanoPi 的比赛频率通信、状态查询和全零软件目标回读可用；不证明 PCA9685 寄存器值、电气 PWM 波形、推进器实际停转或水下运动方向。正式比赛前仍建议完成至少 30 分钟全零浸泡压测和受控断网看门狗测试。
