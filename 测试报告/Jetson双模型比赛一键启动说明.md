# Jetson 双模型比赛一键启动说明

## 部署状态与用途

入口文件是 `AUV-Connection-Available/auv_competition.sh`，配套文件是同目录的 `competition_zero_check.py`。这是新双模型入口，不是旧的 `run_competition.sh`。

2026-10-08 已恢复连接并部署至 Jetson `192.168.137.1`，ROS 2 Humble 构建通过。已在推进器动力电源断开的条件下，完成相机/双模型启动、NanoPi 全链路未解锁定时测试、手动停车与重复启动拦截；没有执行真实比赛解锁或非零运动。底层服务为 NanoPi `192.168.137.219:8888`。详情见同目录 `双模型一键启动部署验证-20261008.md`。

系统流程：ZED 左目 RGB / 注册深度 → 球检测模型与门框四点 Pose（按任务状态调度）→ 比赛状态机与视觉伺服 → 20 Hz 运动指令桥 → NanoPi。没有修改 NanoPi 的通信协议。视觉升沉不等于绝对水深保持；目标丢失时不能继续依赖视觉定深。

## 部署文件

以下命令在开发电脑的项目根目录执行，不是在 Jetson 执行；目标地址当前为 `192.168.137.1`，用户名为 `auv`。

先在 Jetson 备份原有 ROS 源码，再上传修改，避免覆盖后无法回退：

```bash
ssh auv@192.168.137.1 'mkdir -p /home/auv/competition_backups; tar -czf /home/auv/competition_backups/control-$(date +%Y%m%d-%H%M%S).tar.gz -C /home/auv/ros2_ws/src auv_competition_control'

scp AUV-Connection-Available/auv_competition.sh \
    AUV-Connection-Available/competition_zero_check.py \
    AUV-Connection-Available/competition_terminal_monitor.py auv@192.168.137.1:/home/auv/

scp AUV-Connection-Available/AUV_ws/src/auv_competition_control/auv_competition_control/dual_model_perception_node.py \
    auv@192.168.137.1:/home/auv/ros2_ws/src/auv_competition_control/auv_competition_control/
scp AUV-Connection-Available/AUV_ws/src/auv_competition_control/launch/dual_model_mission.launch.py \
    auv@192.168.137.1:/home/auv/ros2_ws/src/auv_competition_control/launch/
scp AUV-Connection-Available/AUV_ws/src/auv_competition_control/config/competition_params.yaml \
    auv@192.168.137.1:/home/auv/ros2_ws/src/auv_competition_control/config/
scp AUV-Connection-Available/AUV_ws/src/rov_thruster_bridge/rov_thruster_bridge/thruster_bridge_node.py \
    auv@192.168.137.1:/home/auv/ros2_ws/src/rov_thruster_bridge/rov_thruster_bridge/
```

然后在 Jetson / FinalShell 终端执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/auv/ros2_ws
colcon build --packages-select auv_competition_control rov_thruster_bridge
```

构建失败时不要继续启动。脚本会自行加载 ROS 环境，无需使用 Conda Python，也无需额外打开 NanoPi SSH 终端。但 NanoPi 的 `auvmaster` 服务必须已运行，RPC 为 `192.168.137.219:8888`。

## 首次验证：不解锁

```bash
bash /home/auv/auv_competition.sh preflight
bash /home/auv/auv_competition.sh start --prepare-only --duration 30
bash /home/auv/auv_competition.sh status
bash /home/auv/auv_competition.sh logs
```

应看到 `READY_DISARMED`；30 秒到时自动清理退出。这会打开 ZED、加载并预热两个模型、启动控制器/通信桥与 rosbag，但不会调用任务解锁服务，只发送零运动指令。IDLE 下模型不会持续处理比赛目标，不能用这个模式测实际推理帧率。第一次硬件验证请隔离推进器动力电源。

## 比赛一键启动

### 出发等待与首次前进（2026-10-09 已部署）

新版状态机和比赛 YAML 已部署到 Jetson，并完成 Humble 构建与隔离影子测试。正式任务当前未启动，尚未验证机器人真实出发运动。

```text
相机/模型/通信就绪 → 操作员解锁
  → START_DELAY：30 秒零运动指令、运动使能关闭
  → DEPARTURE_FORWARD：前进 5 秒，其他轴为零
  → SEARCH_BALL：开始原有搜球/对准/撞球流程
```

30 秒从控制器成功解锁、连接可用后进入 START_DELAY 开始计时，不从输入命令或相机加载开始计时，因此冷启动总等待通常超过 30 秒。使用单调时钟状态计时，不阻塞 ROS 回调；20 Hz 下时长精度受一个控制周期影响。

配置在 `auv_competition_control/config/competition_params.yaml` 的 `competition_controller.ros__parameters`：

```yaml
startup_delay_s: 30.0
departure_duration_s: 5.0
departure_surge: 20.0
```

`departure_surge` 是上层 surge 指令，不是米/秒、RPM 或 PWM 占空比。只允许正值且不超过 `maximum_approach_surge`。通用 PlannerConfig 默认不启用出发段（两项时长为零），比赛 launch 加载这份 YAML 才启用 30 秒/5 秒。`--prepare-only` 不解锁，因此不启动等待倒计时，也不会前进。

等待中掉线：输出零指令，重连后重新完整等待 30 秒。首次前进中掉线：进入 FAILSAFE 并保持零指令，重连不自动继续/重复盲目前进，需要操作员重新启动或重新解锁。正常完成出发段后，普通重连不会重做这 5 秒；原有其他任务阶段的重连策略尚未整体改造。

出发段不会因为提前看到球就跳过等待或前进。5 秒不等于固定距离，期间没有新增障碍检测或视觉定深；没有水深传感器时，目标不在画面中仍不能保证水深。应在水池、限制功率与物理急停条件下由操作员验证，不能在陆地带电空转测试。

Jetson 上修改时长时，编辑 `/home/auv/ros2_ws/src/auv_competition_control/config/competition_params.yaml`，然后在任务停止期间构建，再启动任务；不是运行时热更新：

```bash
nano /home/auv/ros2_ws/src/auv_competition_control/config/competition_params.yaml
source /opt/ros/humble/setup.bash
cd /home/auv/ros2_ws
colcon build --packages-select auv_competition_control
```

两项时长支持小数且必须非负；设置为 `0.0` 跳过对应阶段。不要在运行中热替换控制代码或修改已安装配置。

启动后，默认终端实时状态每秒显示等待倒计时和首次直行状态，例如：

```text
状态=START_DELAY | 出发等待：剩余 30 秒（设定 30 秒），零指令等待
状态=START_DELAY | 出发等待：剩余 29 秒（设定 30 秒），零指令等待
状态=DEPARTURE_FORWARD | 直行中：剩余 5 秒（设定 5 秒），前进指令 surge=20（非实测速度）
状态=DEPARTURE_FORWARD | 直行中：剩余 4 秒（设定 5 秒），前进指令 surge=20（非实测速度）
状态=SEARCH_BALL
```

显示自动跟随 YAML 中的时长，不写死为 30/5；剩余秒数向上取整。倒计时由控制器单调时钟计算，通过 `/auv/mission_progress` 发送；重新打开监视窗口不会重置倒计时。状态或计时数据过期时不显示旧剩余时间。控制器日志也会按剩余秒数变化打印中文提示，`logs` 中可查看。

如果之前使用了 `--detach`，或关掉了显示窗口，可单独执行：

```bash
bash /home/auv/auv_competition.sh monitor
```

`monitor` 仅显示，不解锁、不重启任务；Ctrl+C 只关闭显示，停车仍需执行 `bash /home/auv/auv_competition.sh stop`。

验证结果：本机 L1 回归 60 项、Jetson L1 控制核心 25 项通过；L2 影子测试实际在 30.032 秒进入前进状态，35.082 秒进入搜索球，输出只在 `/auv/shadow/*`。没有启动桥或给 NanoPi 下发运动指令，不属于真实运动验证。备份及日志：`/home/auv/competition_backups/startup-20261009/`。

本次发现开机系统时间回到 1970 年，增量构建误用旧安装文件；已将该包旧 build/install 移到上述备份并完整重建，核对源码与安装文件一致。以后重启后检查 `date`，构建后核对安装配置，不能仅凭构建成功判断更新已生效。

操作员确认机器人在水中、场地清空、物理急停可用后，选择目标球色：

```bash
# 红球任务
bash /home/auv/auv_competition.sh start --allow-motion --target red

# 蓝球任务（二选一，不要同时启动）
bash /home/auv/auv_competition.sh start --allow-motion --target blue
```

`--allow-motion` 代表允许真实运动。默认启动前进程、相机、模型、连接、指令唯一性检查；双模型预热完成后才解锁。不自动开机运动，不自动重启并再次解锁。可用 `--duration 600` 在解锁后 600 秒自动停止。

默认模型：

- 检测：`/home/auv/jetson-zed-yolo/models/competition_latest.pt`
- 门框 Pose：`/home/auv/jetson-zed-yolo/models/gate_pose.pt`

更换模型时添加 `--detector /完整路径/模型.pt --pose /完整路径/模型.pt`，类别与四角顺序必须符合已有感知节点约定。门框名义尺寸配置为 0.70 × 0.50 m；规则是约值，PnP 默认保持关闭，不能把图像方位当作已标定门框三维姿态。

## 查看与停止

2026-10-08 增加实时终端显示：`start` 成功后默认每秒打印一次系统阶段、比赛状态、NanoPi 连接状态、运动是否解锁，以及 NanoPi 返回的航向角 yaw。监视程序只订阅状态/诊断，不创建执行器话题发布者、不额外请求 RPC、不更改控制频率。

示例格式（只是格式示例，不是实测读数）：

```text
阶段=RUNNING | 状态=ALIGN_GATE | NanoPi=在线 | 运动=已解锁 | 陀螺仪[NanoPi航向yaw]=+12.34° 有效 age=0.10s
```

这里 yaw 的单位是度，是航向角，不是陀螺仪角速度。当前 NanoPi 协议只提供这一角度，没有 roll/pitch 或三轴角速度；监视器不虚构这些数据，也不混用 ZED IMU。传感器失效显示“无效”，诊断超过 2 秒未更新或遥测超过 0.5 秒显示“无数据/过期”，不会沿用上一次角度冒充实时值。运动显示代表桥的使能状态，不代表实测推进器转速。

### 目标坐标与距离

实时状态下面增加一行“感知目标”，按当前搜球/球对准/球靠近或搜门/门对准/门靠近阶段显示最近一帧的有效候选。它是只读感知观测，不代表目标已经通过控制器的连续帧确认，也不代表发生了碰撞。

球的示例（仅示例）：

```text
感知目标=red_ball [ZED双目表面深度] 相机坐标x=+0.300 y=+0.400 z=1.200m 前向距离=1.200m 直线距离=1.300m frame=zed_left_camera_optical_frame 接收age=0.10s
```

- 坐标基于 ZED 左目光学坐标系，单位米：x 向右、y 向下、z 向前；不是运动指令 xyz，也不是机器人本体坐标。
- 前向距离是 z，直线距离为 `sqrt(x*x+y*y+z*z)`。球的数值来自检测框中央区域的双目深度中位数，主要反映可见表面，不是精确球心或机器人前端间隙。
- 门框有有效 PnP 三维解才打印相机坐标和米制距离，来源标注“门框PnP估计”。目前 PnP 默认关闭，会显示“Pose二维”中心归一化误差和“三维坐标/距离不可用”，不能把归一化中心当成米制坐标。
- 最近收到空观测或无效观测立即清除显示；超过 0.35 秒未收到新观测显示“未收到/已过期”。此 age 是终端监视器的接收年龄，不是曝光时间或完整端到端延迟。
- `HIT_BALL`、`RETREAT`、`PASS_GATE` 等定时阶段不继续显示旧坐标；`--prepare-only` 的 IDLE 阶段不做目标推理，显示“当前阶段不进行视觉伺服”是正常的。

该更新只增加感知话题订阅，没有重启正在运行的比赛或修改运动指令。现有监视进程需要 Ctrl+C 关闭后重新执行 `monitor` 才加载新版；新启动任务自动使用新版。

`Ctrl+C` **仅关闭终端显示，不停止后台比赛任务**。SSH 断开也不会停车；停车必须执行 `stop`。若需要启动成功立即返回，添加 `--detach`；之后可重新打开显示：

```bash
bash /home/auv/auv_competition.sh start --prepare-only --duration 30 --detach
bash /home/auv/auv_competition.sh monitor
```

```bash
bash /home/auv/auv_competition.sh status
bash /home/auv/auv_competition.sh logs
bash /home/auv/auv_competition.sh monitor
bash /home/auv/auv_competition.sh stop
```

启动后与 SSH 会话脱离，断开 FinalShell 不会停车。关闭终端前若不需要继续运行，必须执行 `stop`。软件停车不是物理急停；机器失联、死机或断电时不能依赖 SSH 命令。

终端监视更新已部署，Jetson 构建和监视节点无动作启动检查通过；本机 27 项相关单元测试通过，覆盖有效/无效 yaw、断流和过期判定。此次更新验证时 NanoPi 8888 服务停止，因此尚未验证新终端界面的真实传感器读数；不要把格式示例或旧的全链路测试当成这次实测。

关键节点退出、启动失败、录包进程退出或到达时长时：监督器尝试锁定任务、停止它拥有的进程组、独立发送三次零运动指令，并检查 NanoPi 六路 `propeller_power` 全零。该反馈是软件功率值，不是实测转速，也不是把 PWM 强改为 1500 µs；底层已校准的 1600 µs 中位保持不变。失败会保留日志并返回非零状态，不能当作已安全停车。

## 日志与调试图

每次运行记录在 `/home/auv/ros2_ws/competition_runs/时间戳/`，最新目录通过 `/home/auv/competition_runtime/latest` 查看，包括 `launch.log`、`supervisor.log`、`arm.log`、`disarm.log`、`final-zero.log`、模型 SHA256 和 rosbag。不要在运行中删除这些文件。

默认不编码调试 JPEG，以减少推理负载。2026-10-10 只读浏览器入口及独立视觉调试模式已部署到 Jetson，Humble 构建、目标机单元测试及真实相机网页链路检查通过；下列命令可直接使用。

### 下水任务中查看实时检测

在符合真实运动条件时，在 Jetson 的第一个 SSH 终端启动任务并开启图像：

```bash
bash /home/auv/auv_competition.sh start --allow-motion --target red --debug-image
```

第二个 SSH 终端启动只读网页服务：

```bash
bash /home/auv/auv_competition.sh preview
```

Windows/岸上电脑浏览器打开 `http://192.168.137.1:8081`。FinalShell 只负责 SSH；看图使用 Edge/Chrome 等浏览器，不需要在 Windows 安装 ROS 或 ZED SDK。需要保持岸上与交换机的有线网络连接，水下 Wi-Fi 不能替代这根链路。

球阶段显示检测框、类别、置信度；只有当前目标球的有效 ZED 深度可作为控制输入。二维框可见但距离无效时，不代表控制器接受了该目标。门阶段显示 Pose 四角和连线；等待、首次直行、定时撞球/穿门阶段仅显示相机图像和状态，不增加推理。预览上限 5 FPS 不代表模型推理速度，检测/控制仍按原流程运行。JPEG 编码和传输会增加负载，下水前须重新测量控制/推理频率。

`preview` 不占用相机，不加载模型，不发运动指令；Ctrl+C 只关闭网页，不停止比赛。真实任务停车仍用 `bash /home/auv/auv_competition.sh stop`。如果任务启动时没加 `--debug-image`，网页会提示等待图像；不要另外启动第二个相机或 YOLO 进程。

### 岸上或下水人工移动机器人，只测试视觉

先确保比赛链、录像程序及其他相机进程均已停止；建议断开推进器动力电源。运行：

```bash
bash /home/auv/auv_competition.sh vision-debug
# 若需要蓝球深度显示：
# bash /home/auv/auv_competition.sh vision-debug target_ball_color:=blue
```

同样打开 `http://192.168.137.1:8081`。此模式仅运行 ZED 来源、双模型和网页，**没有控制器、NanoPi 桥或解锁服务**，不依赖 NanoPi 在线。图像同时绘制检测模型的二维框及门框四角；观测话题隔离在 `/auv/vision_debug/*`，DDS domain 97，与比赛 domain 96 分开。不能和正式比赛同时运行。Ctrl+C 关闭整条视觉调试链并释放相机；此启动方式为前台运行，不保证 SSH 断开后继续。

### 网络与排障

- 网页打不开：确认 IP、网络、端口 8081，以及 `preview`/`vision-debug` 进程是否运行；端口已占用时先确认占用者，不要直接杀其他程序。
- 页面提示等待/断流：网页仍在线，但没有收到最近 2 秒内的 JPEG；不会继续显示旧画面。检查相机、模型日志与 `--debug-image`。
- 改端口：`preview --ros-args -p port:=8082`，或 `vision-debug web_port:=8082`，浏览器对应改为 8082。
- 服务默认监听 `0.0.0.0`，没有鉴权，仅用于可信岸上局域网，不能暴露到公网。若仅通过 SSH 隧道查看，用 `preview --ros-args -p host:=127.0.0.1`，再配置 FinalShell 本地端口转发。

已部署文件：新版 `auv_competition.sh`、`setup.py`、`dual_model_perception_node.py`、新增 `perception_web_viewer.py` 和 `launch/vision_debug.launch.py`；同时保护相机和网页清理阶段，避免重复 SIGINT 中断退出。备份在 `/home/auv/competition_backups/web-preview-20261010/`，含旧源码、启动脚本及 build/install 包，可回退。

验证范围：Jetson L1 相关测试 9 项通过，L2 Humble 构建及独立视觉启动通过；岸上主机通过 HTTP 收到并查看了 1280×720 的真实 ZED 图像，两个模型预热完成，网页持续接收新帧。测试场景没有比赛球/门，未验证目标检测准确率或水下画质，也未测量加入预览后正式任务的推理/控制频率；没有启动控制器/通信桥或发送运动指令。测试日志存于上述备份目录。

本机测试属于 L1（无真实硬件）：启动脚本测试与控制核心、门框几何、RPC 客户端回归合计 38 项通过，包含启动失败后的 EXIT 清理回归。另完成 Jetson 构建/启动检查，以及推进器动力隔离条件下的 L4 未解锁验证：30 秒采样中 617 条控制指令均为零、频率 20.0 Hz，617 条运动使能均为 false；结束后四节点正常退出、rosbag 封尾、六路底层功率反馈为零。尚未完成正式解锁、L5/L6 运动或水下全流程验证；预热耗时不等于检测帧率。
