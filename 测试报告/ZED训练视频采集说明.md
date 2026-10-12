# ZED 水下训练视频采集说明

面向 Jetson 与 Windows FinalShell 的现场操作版见：[Jetson-ZED录制抽帧与FinalShell下载说明.md](./Jetson-ZED录制抽帧与FinalShell下载说明.md)。

## 1. 脚本与数据位置

- Jetson 部署目标：`/home/auv/record_training_video.sh`
- 实时预览录制器：`/home/auv/record_zed_preview.py`
- 默认数据根目录：`/home/auv/zed_datasets`
- 每次采集目录：`时间-场景标签/`
- 原始视频：`raw.svo2`
- 元数据：`metadata.txt`
- 运行日志：`record.log`

脚本只启动 ZED 原始数据录制，不启动 YOLO、ROS 2 比赛控制器、推进器桥或 NanoPi 程序，也不会发送运动指令。SVO2 保留左右目原始画面以及 ZED 传感器元数据，后续可重复调整深度参数并导出训练图片。

## 2. 常用命令

通过 FinalShell 登录 Jetson（当前有线地址 `192.168.137.1`）后执行：

```bash
# 录制 10 分钟；SSH 中断后仍会继续
bash /home/auv/record_training_video.sh start \
  --tag pool_ball_gate \
  --duration 600

# 查看是否正在录制、文件大小和剩余空间
bash /home/auv/record_training_video.sh status

# 提前停止并让 SVO2 正常封尾
bash /home/auv/record_training_video.sh stop

# 列出所有采集批次
bash /home/auv/record_training_video.sh list
```

`pool_ball_gate` 可以改为本次录制的英文标签。

如需在录制的同时用 Windows 浏览器实时查看：

```bash
bash /home/auv/record_training_video.sh start \
  --tag pool_ball_gate \
  --duration 600 \
  --web-preview
```

然后在 Windows 打开 `http://192.168.137.1:8080`。该页面是约 8 FPS 的左目取景预览，默认不画 YOLO 检测框。预览模式使用无损 SVO2，占用空间明显更大；10 分钟任务会按最大约 36 GiB 数据加 10 GiB 安全余量检查空间。普通录制不加 `--web-preview`，仍使用更节省空间的 JPEG SVO2。

`--duration 0` 表示持续录制，直到手动执行 `stop`。定时录制按“时长×帧率”控制有效帧数，不把相机初始化时间算进录像时长，同时保留额外 30 秒异常超时保护。默认使用 `HD720@30 FPS` 和有损 JPEG（压缩模式 2），避免依赖这台 Jetson 当前不可用的 H.264 硬件编码。JPEG 模式仍保存双目画面并可在回放时重新计算深度，更适合目标检测训练素材。默认磁盘剩余空间低于 10 GiB 时拒绝启动。场景标签只能使用英文字母、数字、点、下划线和短横线。

## 3. 推荐采集流程

1. 推进器断电或拆桨，将机器人固定可靠；本阶段只采数据。
2. Jetson 和 ZED 上电，SSH 登录后运行 `status`，确认没有旧录像。
3. 根据场景启动录像，观察启动结果中的保存路径。
4. 下水后让红球、蓝球和门框覆盖远、中、近距离，以及画面中心和四周。
5. 同时采集无目标、黑色物体、反光、水泡、池壁、人员等负样本，重点覆盖当前会被误判为蓝球的黑色目标。
6. 每个场景建议录制 2～5 分钟并更换光照、距离、角度和水体浑浊程度，不要只录一段很长且高度重复的视频。
7. 出水后执行 `stop`，看到 `Recorder stopped cleanly` 后再关闭 Jetson 电源。
8. 用 `status` 确认 `STOPPED`，并检查 `raw.svo2` 大小不是 0。

## 4. 安全与故障处理

- 不要同时运行比赛程序或 `detect3d.py`。ZED 相机通常只能被一个进程独占，脚本检测到冲突会拒绝启动。
- 不要直接断电结束录制，否则 SVO2 索引可能未写完。优先执行 `stop`。
- `FAILED: ZED recording did not start`：查看本次目录中的 `record.log`，并检查 ZED USB 连接。
- `less than ... GiB is available`：先转移旧数据，不建议关闭空间保护；短时排障可显式传入 `--min-free-gb 1`。
- `a recording is already active`：先运行 `status`，确实需要结束时再运行 `stop`。

## 5. 当前验证状态

- 已完成 Bash 语法检查。
- 已用模拟 ZED 进程验证启动、状态查询、SSH 脱离、手动优雅停止、定时停止和状态清理。
- 已部署到 `192.168.137.1:/home/auv/record_training_video.sh`。
- 已部署实时预览录制器到 `192.168.137.1:/home/auv/record_zed_preview.py`。实机联调录制 20 秒得到精确 600 帧、1280×720@30 FPS 的 SVO2；岸上主机成功访问 `http://192.168.137.1:8080`，获取 640×360 JPEG 实时帧和连续 MJPEG 流。另一次手动停止测试正常封尾并由 SDK 回放 710 帧。录像结束后端口和相机均正常释放。启动脚本会等待 ZED 和网页服务明确就绪后才输出成功提示。
- ZED 2i 实机完成两次 10 秒录制：无损 ZSTD 文件约 438 MiB、253 帧；JPEG 文件约 184 MiB、251 帧。两者均由 ZED SDK 成功回放打开，最终默认采用容量更合理的 JPEG 模式。
- Jetson 初始系统时间停留在 1970 年，现已从调试电脑同步至 2026-09-26。最初的定时实现把相机初始化计入时长，5 秒只录到 100 帧；现已改为按目标帧数停止，最终回归得到准确的 5 秒、150 帧 SVO2。
- Jetson 的硬件 RTC 未能保存本次时间，完全断电重启后仍可能回到 1970 年。脚本检测到年份早于 2024 时会明确警告，但不会阻止应急录像；正式采集前应先执行 `date --iso-8601=seconds` 检查。

实机验证命令：

```bash
bash /home/auv/record_training_video.sh start \
  --tag dry_smoke \
  --duration 10 \
  --min-free-gb 1

sleep 15
bash /home/auv/record_training_video.sh status
bash /home/auv/record_training_video.sh list
```

## 6. 在岸上主机查看和下载

本地提供两个辅助程序：

- `AUV-Connection-Available/export_svo.py`：在 Jetson 上把 SVO2 左目画面导出为普通 MP4，也可按间隔导出 YOLO 训练图片。
- `AUV-Connection-Available/fetch_zed_recording.sh`：在岸上 Linux 主机选择最新批次、触发安全转码并下载 MP4；添加 `--with-svo` 时同时下载大型原始文件。

Jetson 再次上电后，先部署本轮离线完善的版本：

```bash
scp AUV-Connection-Available/record_training_video.sh \
    AUV-Connection-Available/export_svo.py \
    auv@192.168.137.1:/home/auv/

ssh auv@192.168.137.1 \
    'chmod +x /home/auv/record_training_video.sh /home/auv/export_svo.py'
```

在岸上 Linux 主机一键获取最新录像的普通 MP4：

```bash
cd /home/cjx/cjx_project/auv比赛/AUV-Connection-Available
./fetch_zed_recording.sh
```

下载结果默认位于：

```text
./zed_recordings/<批次名称>/preview.mp4
```

可使用 VLC、系统视频播放器或 OpenCV 打开。下载指定批次或同时下载原始 SVO2：

```bash
./fetch_zed_recording.sh --session 20260926-163740-final_default
./fetch_zed_recording.sh --with-svo
```

在 Jetson 上额外导出训练图片：

```bash
/usr/bin/python3 /home/auv/export_svo.py \
  /home/auv/zed_datasets/<批次名称>/raw.svo2 \
  --video /home/auv/zed_datasets/<批次名称>/preview.mp4 \
  --images-dir /home/auv/zed_datasets/<批次名称>/images \
  --every-n 10
```

Jetson 若显示 `(base)`，表示 Miniforge 环境已激活；不要用该环境的 `python3` 运行 ZED 工具。ZED SDK、OpenCV 和 NumPy 位于系统 Python 3.10，必须显式使用 `/usr/bin/python3`，或直接执行具有固定系统 Python shebang 的脚本。

导出器先写入 `preview.partial.mp4`，全部帧完成后才原子改名并生成 `preview.mp4.complete`。岸上脚本只有同时检测到视频和完成标记才会下载；转码中断电时会在下次连接后自动重建，不会把半成品当作有效视频。

本机在没有 ZED SDK 的情况下通过模拟 ZED API 验证：12 帧输入可导出 12 帧 MP4、每 3 帧抽样得到 4 张 JPEG；模拟第 2 帧读取失败时不会残留可播放文件、临时文件或完成标记。

2026-09-26 最终真机全流程验证：一键脚本自动选择 `20260926-171500-duration_check`，在 Jetson 导出并下载 MP4；主机 OpenCV 解码结果为 150 帧、30 FPS、5.000 秒、1280×720，文件约 2.57 MiB。中间帧目视确认清晰、方向正常。全过程没有启动比赛程序、推进器桥或 NanoPi。

## 7. 一键抽取并下载 YOLO 训练图片

岸上电脑脚本：

```text
AUV-Connection-Available/fetch_yolo_frames.sh
```

Jetson 抽帧程序：

```text
/home/auv/extract_svo_frames.py
```

也可在 Jetson 的录像目录中直接执行。即使终端显示 `(base)`，也要显式使用系统 Python：

```bash
cd /home/auv/zed_datasets/20260926-220904-pool_ball_gate

/usr/bin/python3 /home/auv/extract_svo_frames.py \
  ./raw.svo2 \
  --output ./yolo_frames_n10_q95 \
  --every-n 10 \
  --jpeg-quality 95
```

不要在没有先赋值的情况下使用 `${session}`，否则路径会展开成错误目录。直接进入目标录像目录后使用 `./raw.svo2` 最不容易输错。

在岸上电脑执行以下命令，会自动选择最新录像、在 Jetson 上抽取左目 JPG，并通过单个 tar 数据流传回电脑：

```bash
cd /home/cjx/cjx_project/auv比赛/AUV-Connection-Available
./fetch_yolo_frames.sh
```

默认参数为每 10 帧抽取一张、JPEG 质量 95。指定抽帧间隔或录像批次：

```bash
./fetch_yolo_frames.sh --every-n 15

./fetch_yolo_frames.sh \
  --session 20260926-171500-duration_check \
  --every-n 10
```

图片默认下载到：

```text
AUV-Connection-Available/yolo_frames/<录像批次>/n10_q95/
```

目录中包含：

```text
frame_000000.jpg
frame_000010.jpg
...
frames.csv       # 图片名、原始帧号、ZED时间戳
.complete        # 完整抽取与传输标志
```

抽帧过程使用临时目录，全部图片和清单写完后才原子改名；断电或网络中断不会把半成品标记为有效数据。重复执行相同批次和参数时会复用已完成的 Jetson/电脑结果。

真机验证：150 帧 SVO2 使用 `--every-n 10` 得到 15 张图片；`20260926-220904-pool_ball_gate` 的 464 帧 SVO2 得到 47 张图片。两批结果均为 1280×720 左目 JPG，全部传回电脑并逐张解码通过。
