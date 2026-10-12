# 视频回放 Jetson—NanoPi 比赛全链测试报告（2026-10-06）

## 结论

使用已采集的 `preview.mp4` 在 Jetson 上分别回放撞球片段和过门片段，两个比赛模型、比赛状态机、正式 ROS 控制话题、Jetson 通信桥、NanoPi JSON-RPC 与六路混控全部连通。两段共处理 1800 帧，每帧控制结果均发布到正式 ROS 接口；通信桥以 20 Hz 取最新结果发送给 NanoPi。未发现异常退出、通信断开或无法回零。

本次证明“录制画面 → 模型 → 状态机 → 四轴控制量 → NanoPi 六路 PWM 软件目标”的接口可以工作。它不是完整任务成功验收：录制视频不会因控制指令改变后续画面，因此状态机只能完成搜索和对准分支，不能据此证明真实机器人能撞到球或穿过门框。

## 安全条件与验证等级

- 推进器动力在测试前已物理断开；Jetson、NanoPi 与控制板保持上电。
- 测试使用真实 `/auv/control_command`、`/auv/motion_enable`、`rov_thruster_bridge` 和 NanoPi `move` 接口。
- 回放脚本只有同时给出 `--enable-nanopi --allow-pwm` 才允许加载输出路径。
- 每段结束发送 8 组关闭使能与全零命令；桥接退出后又独立发送全零并读取六路结果。
- 最终六路结果全部为 0，Jetson 无回放或桥接进程残留。

验证等级为 **L4**：真实 Jetson、NanoPi 与软件 PWM 链路工作，但推进器动力断开，没有真实转速、推力或机器人运动。

## 测试配置

- Jetson：`192.168.137.1`
- NanoPi：`192.168.137.219:8888`
- ROS 2 Humble，`ROS_DOMAIN_ID=96`
- 视频：`/home/auv/zed_datasets/20260928-153741-test1/preview.mp4`
- 球/门检测模型：`/home/auv/jetson-zed-yolo/models/competition_latest.pt`
- 门框 Pose 模型：`/home/auv/jetson-zed-yolo/models/gate_pose.pt`
- 推理：CUDA FP16，`imgsz=640`
- 回放脚本：`/home/auv/test_closed_loop_video_web.py`
- 输出目录：`/home/auv/video_replay_runs/20261006/`

## 撞球阶段

- 视频范围：0～30 秒，900 帧。
- 状态：`SEARCH_BALL` 437 帧，`ALIGN_BALL` 463 帧。
- 状态转移：14.600 秒时 `SEARCH_BALL → ALIGN_BALL`。
- 463 帧获得红球观测；437 帧无选中目标。
- 控制范围：`heave -17.34～+35.00`，`yaw -13.43～+19.43`；该片段未满足稳定对准条件，因此未产生前进 `surge`。
- 墙钟处理时间约 58.8 秒，含模型推理、画框、MP4 编码、JPEG 调试帧、ROS 发布与 NanoPi 回读，约 **15.3 FPS**。

## 过门阶段

- 视频范围：95～125 秒，900 帧；通过测试专用相位注入从 `SEARCH_GATE` 开始，并非伪造生产状态转移。
- 状态：`SEARCH_GATE` 536 帧，`ALIGN_GATE` 364 帧。
- 在目标有效/失效时多次在搜索与对准之间切换；未满足连续稳定对准条件，未进入 `APPROACH_GATE`。
- 控制范围：`sway -17.16～+13.43`、`heave -20.86～+19.57`、`yaw -20.00～+20.00`；对准阶段按设计不输出前进 `surge`。
- 墙钟处理时间约 95.5 秒，含普通检测、条件 Pose、画框、MP4/JPEG 编码、ROS 与 NanoPi 回读，约 **9.4 FPS**。

门阶段低于此前设定的 10 Hz 目标约 6%。该数据包含比赛程序正常不需要的逐帧 MP4、JPEG 调试编码，不能直接等同于正式无录像模式；但余量不足是真实风险。此前纯推理协调模式为 20.63 FPS、真实 ZED 全链约 11.63 Hz。正式下水前仍应以“真实 ZED + 双模型 + 正式控制器 + NanoPi、关闭录像/JPEG”再做一次持续帧率验收。

## NanoPi 六路证据

桥接共记录 396 组六路 PWM 软件目标回读，其中 377 组至少一路非零，19 组全零。两阶段合并范围：

| 通道 | 最小偏移 | 最大偏移 |
|---|---:|---:|
| left_up | -94 | +68 |
| right_up | -87 | +68 |
| left_middle | -59 | 0 |
| right_middle | 0 | +48 |
| left_down | -62 | +52 |
| right_down | -58 | +61 |

最后一组回读为六路全零；桥接停止后独立协议检查再次得到六路全零。这里的数值是相对当前已校准停止基准的软件偏移，不是 RPM、绝对脉宽或示波器波形。

## 已知限制与下一步

1. MP4 没有 ZED 配准深度，球距离由已知球直径和框像素大小近似；正式比赛仍应优先使用 ZED 有效距离，并对异常距离做滤波/降级。
2. 录制视频是开环输入，控制器无法改变后续画面；它适合验证接口、异常、状态分支和 PWM 输出，不适合验证闭环任务成功率。
3. 本片段没有覆盖 `APPROACH_BALL → HIT_BALL → RETREAT` 与 `APPROACH_GATE → PASS_GATE → DONE`。这些分支已有无执行器模拟测试，但仍需水池实机覆盖。
4. 下一项应为系留、低限幅水池测试：先只开升沉稳定，再测球对准，最后测门框对准；每一步均保留人工急停和失联回零检查。

## 留存文件

- 本地撞球控制记录：`测试报告/ball_commands.csv`
- 本地过门控制记录：`测试报告/gate_commands.csv`
- 本地桥接回读日志：`测试报告/video_replay_bridge_20261006.log`
- Jetson 画框录像：`/home/auv/video_replay_runs/20261006/ball_replay.mp4`、`gate_replay.mp4`
