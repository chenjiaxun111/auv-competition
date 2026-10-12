# 双模型感知与 Pose 过门改造说明

## 运行流程

1. `SEARCH_BALL / ALIGN_BALL / APPROACH_BALL`：运行普通 YOLO11n 检测模型，只选择当场指定的红球或蓝球，结合 ZED 配准深度生成球的相机坐标 XYZ。
2. `SEARCH_GATE`：先用普通检测模型寻找 `gate` 候选；存在候选时才运行门框 YOLO11n-Pose。
3. `ALIGN_GATE / APPROACH_GATE`：仅运行 Pose，固定四点顺序为左上、右上、右下、左下。控制量由门中心误差、水平方位、门框图像宽度和可选 PnP 距离产生。
4. `HIT_BALL / RETREAT / PASS_GATE`：执行受限的定时动作，不运行神经网络。

## ROS 话题

- 输入：ZED 左目矫正图像、配准深度、CameraInfo 以及 `/auv/mission_state`。
- 球观测：`/auv/perception/ball_observations`。
- 门框 Pose：`/auv/perception/gate_pose_observations`。
- 调试画面：`/auv/perception/debug/image/compressed`。
- 推理诊断：`/auv/perception/diagnostics`。
- 发往旧通信桥的接口保持不变：`/auv/control_command`、`/auv/motion_enable`、`/auv/target_depth_m`。视觉升沉模式使用 `control_command.force.z`，不发布绝对深度目标。

## 无深度传感器时的视觉升沉

- 当前参数使用 `vertical_control_mode: visual_heave`，启动后跳过依赖绝对深度反馈的 `DIVE`，从 `SEARCH_BALL` 开始。
- 球阶段以目标相对相机光轴的垂直夹角为误差；门阶段以四角中心相对画面中心的归一化纵向误差为误差。
- 目标在画面下方时请求正升沉，目标在画面上方时请求负升沉；最终物理方向由桥接层 `axis_sign.heave` 决定，首次台架必须核对。
- 控制器包含死区、PD、幅值限制和变化率限制。视觉目标超时、搜索、撞击、后退和盲穿阶段均把升沉请求置零。
- 该方案只能让机器人相对目标上下对准，不能知道水面距离、池底距离或绝对水深。下水前应把机器人放到接近工作深度，并把浮力配平到接近中性。
- 当前保守上限 `maximum_visual_heave=40`；桥接层 `max_heave=250` 时，传给 NanoPi 的归一化 `move(z=...)` 最大约为 `0.16`。

## 安全启动（Jetson 影子模式）

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash

ros2 launch auv_competition_control dual_model_mission.launch.py \
  shadow_mode:=true \
  auto_start:=true \
  vertical_control_mode:=visual_heave \
  target_ball_color:=red \
  detector_model_path:=/home/auv/jetson-zed-yolo/models/competition_latest.pt \
  gate_pose_model_path:=/home/auv/jetson-zed-yolo/models/gate_pose.pt
```

启动前必须先让 ZED ROS 节点发布左目图像、配准深度和 CameraInfo。影子模式不启动 NanoPi 桥，不创建真实执行器话题。

Jetson 当前没有 ZED ROS Wrapper，可改由新增的 ZED SDK 直连节点提供相同话题：

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=97

ros2 launch auv_competition_control dual_model_mission.launch.py \
  start_zed_sdk_source:=true \
  shadow_mode:=true \
  auto_start:=true \
  vertical_control_mode:=visual_heave \
  inference_device:=0 \
  use_half_precision:=true \
  target_ball_color:=red \
  detector_model_path:=/home/auv/jetson-zed-yolo/models/competition_latest.pt \
  gate_pose_model_path:=/home/auv/jetson-zed-yolo/models/gate_pose.pt
```

该节点直接调用 `pyzed.sl.Camera`，同步发布：

- `/zed/zed_node/left/image_rect_color`：左目矫正图，`bgr8`；
- `/zed/zed_node/depth/depth_registered`：左目配准深度，`32FC1`，单位米；
- `/zed/zed_node/left/camera_info`：ZED SDK 当前分辨率下的矫正内参。

ZED 同一时刻只能有一个相机所有者。启动前必须停止 `detect3d.py`、SVO2 录制、网页预览及其他 ZED 程序。

## Jetson GPU 推理环境

- 2026-10-06 已将用户目录中的 CPU 版 PyTorch 替换为 JetPack 5/CUDA 11.4 对应的 AArch64 构建：`torch 2.2.0`、`torchvision 0.17.2+c1d70fe`。
- GPU 自检结果：`torch.version.cuda == 11.4`、`torch.cuda.is_available() == True`，设备名为 `Orin`，FP16 CUDA 矩阵运算通过。
- 两个比赛模型均通过 GPU 加载和推理烟雾测试。640 输入下，球检测模型稳态约 28.5 ms，门框 Pose 模型稳态约 30.6 ms；这些是单模型合成图测试，不等同于包含 ZED 深度和 ROS 传输的全链帧率。
- 变更前包清单、原运行信息、校验后的安装包以及退出竞态修复前源码均保存在 `/home/auv/deployment_backups/python-ml-before-cuda-20261006/`。
- Jetson 当前是 Ubuntu 22.04/Python 3.10 与 L4T R35.5.0 的定制组合，不是 JetPack 5.1.3 默认的 Ubuntu 20.04/Python 3.8 组合。以后不要直接安装 PyPI 的通用 AArch64 CPU 包，也不要照搬官方 `cp38` wheel 覆盖当前环境。

## 必须标定的参数

- `gate_stop_width_norm`：无 PnP 时由视觉接近切换到盲穿的门框宽度阈值。
- `gate_commit_width_norm`：近门丢角后允许盲穿的最小历史宽度。
- `gate_no_range_surge`：仅使用图像尺度接近门框时的前进上限。
- `gate_width_m / gate_height_m`：门框内开口真实尺寸；未测量和未完成水下标定前，保持 `enable_gate_pnp:=false`。
- `pass_surge / pass_duration_s`：必须在约束机体和水池中从小到大调整。

## 当前验证边界

- L1：当前控制与通信两包源码测试 58/58 通过；其中包含视觉升沉、ROS 节点传播、ZED CameraInfo 和后台推理线程安全退出测试。
- L2：本机 ROS 2 Jazzy 包构建及 launch 解析通过。
- 2026-10-06 已部署至 Jetson `/home/auv/ros2_ws`，Humble 构建成功；通信包 28/28、控制包显式 pytest 25/25 通过。隔离域中成功加载 `competition_latest.pt` 与 Pose v2，并确认 `shadow_mode=true` 时真实 `/auv/control_command`、`/auv/motion_enable` 均不存在，退出后无节点残留。
- 新增 ZED SDK 直连 ROS 图像源后，本机完整测试 55/55、Jetson/Humble 控制包 27/27 通过。真实 ZED 2i 影子验证得到左目约 9.7 Hz、配准深度约 7.9 Hz、有效深度像素约 67.3%；CameraInfo 为 1280×720，`fx=fy=520.96`，三路消息使用同一左目光学坐标系。
- 真实 ZED 测试期间影子状态机进入 `SEARCH_BALL`，感知消息持续更新；现场无球目标，因此未验证真实画面的非零视觉升沉。真实控制话题始终不存在，NanoPi 程序和 8888 端口始终未启动，退出后无相关进程残留。
- CUDA 恢复后的真实 ZED 影子复测得到：左目约 10.38 Hz、配准深度约 10.71 Hz、调试图和推理诊断约 11.63 Hz，末帧全链推理约 78 ms。模型首次导入和加载约需 24 秒，启动阶段状态机会保持无目标搜索。
- 复测画面中出现过一次短暂红球误检，影子状态机从 `SEARCH_BALL` 进入 `ALIGN_BALL` 后又退回搜索；这不能作为真实目标验证结果，反而说明仍需增加水下负样本，并继续使用时序确认与置信度门限抑制误检。
- 实测停止信号曾暴露后台推理线程在 ROS 上下文失效后继续发布的竞态；现已增加上下文/停止状态检查和 3 个回归测试。Jetson 重新构建成功，控制包 30/30 通过，复测退出时三个节点均干净结束，无异常栈。
- L3：无执行器模拟链完整到达 `DONE`；真实训练模型对离线红球帧和门框帧均产生有效观测。
- 离线视频烟雾测试确认绝对深度目标为空且 NanoPi 发送关闭；所选 20 帧仅出现一个未通过几何校验的 Pose 候选，未用于证明真实视频升沉效果。
- 当前已经不依赖 ZED ROS Wrapper，由自研 SDK 节点发布左目、配准深度和 CameraInfo。CUDA 推理已恢复，但真实全链约 11.6 Hz，仍低于配置的 15 Hz；控制器必须以消息时间戳和超时机制工作，不能假定固定 15 Hz。
- 未完成：在真实球/门画面下验证持续稳定观测和非零视觉升沉、升沉方向台架核对、断开动力后的 NanoPi 全链验证和水池调参。

## 2026-10-06 机载版本提示

- NanoPi 当前工程为 `/home/pi/auv-2026`，不再是旧版 `/home/pi/auvmaster-2`；本轮没有启动或修改 NanoPi。
- 新底层仍兼容 `move(x,y,z,yaw)`、`status`、`pid_runtime_status` 和 `propeller_power`，所以 Jetson 通信协议暂时无需调整。
- 底层停止脉宽已改为 1600 us、PCA9685 为 47 Hz、通道为 CH0～CH5，并增加实测死区补偿、横滚闭环和水平混控系数。旧的 1500 us PWM 验收结论不可继续沿用。
- Jetson 部署前备份位于 `/home/auv/deployment_backups/auv_competition_control-before-visual-heave-20261006-144605.tar.gz`。
- ZED SDK 图像源部署前备份位于 `/home/auv/deployment_backups/auv_competition_control-before-zed-source-20261006-145945.tar.gz`。

## MP4 离线闭环调试页面

该页面直接调用比赛使用的 `CompetitionPlanner`，显示模型检测、状态机和计划运动指令，但不导入 NanoPi 通信桥，因而不会向推进器下发数据。

```bash
cd /home/cjx/cjx_project/auv比赛

/home/cjx/miniconda3/envs/auv_env/bin/python3 \
  训练集/test_closed_loop_video_web.py \
  --video 训练集/preview.mp4 \
  --start-phase gate \
  --start-seconds 95
```

随后打开 `http://127.0.0.1:8767`。页面显示：

- 当前比赛状态，例如 `SEARCH_GATE`、`ALIGN_GATE`、`APPROACH_GATE`、`PASS_GATE`；
- 控制器计划的 `surge / sway / heave / yaw`；视觉升沉模式下 `target_depth_m` 显示为禁用；
- 门框中心误差、宽度和方位角；
- 明确固定为关闭的 `NanoPi transmission` 状态。

结果保存在：

- `训练集/video_test_results/preview_closed_loop.mp4`；
- `训练集/video_test_results/preview_closed_loop_commands.csv`。

离线 MP4 不含 ZED 注册深度。球距离在此页面中由框尺寸近似估算，只用于检查状态机，不得代替比赛运行时的 ZED 深度结果。门框控制不依赖该球距离估算。
