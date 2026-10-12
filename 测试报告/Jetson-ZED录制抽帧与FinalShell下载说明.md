# Jetson ZED 录像、抽帧与 FinalShell 下载说明

本文用于比赛现场操作，覆盖以下流程：

```text
ZED 录像 → 安全停止 → 指定录像抽帧 → Windows FinalShell 下载图片
```

## 1. 连接信息与脚本位置

Jetson SSH 信息：

```text
IP：192.168.137.1
端口：22
用户名：auv
密码：请向设备管理员获取，不在仓库中保存
```

Jetson 需要使用的脚本：

```text
/home/auv/record_training_video.sh   # 录制和管理 SVO2
/home/auv/extract_svo_frames.py      # 从 SVO2 抽取左目 JPG
```

数据根目录：

```text
/home/auv/zed_datasets/
```

## 2. 上电后的录制前检查

通过 FinalShell 登录 Jetson 后，先检查系统时间：

```bash
date --iso-8601=seconds
```

Jetson 的硬件 RTC 目前不能在完全断电后保存时间。如果显示 1970 年，应先手动设置，例如：

```bash
sudo date -s '2026-09-27 10:30:00'
```

然后检查相机和录像状态：

```bash
lsusb | grep -iE '2b03|zed'
bash /home/auv/record_training_video.sh status
```

正常情况应看到 ZED 2i 的两个 USB 设备，并显示：

```text
Recorder: STOPPED
```

## 3. 开始录制

以下命令录制 10 分钟：

```bash
bash /home/auv/record_training_video.sh start \
  --tag pool_ball_gate \
  --duration 600
```

参数说明：

- `--tag`：场景标签，会出现在录像目录名中，可以修改。
- `--duration`：有效录像时长，单位为秒；`600` 表示 10 分钟。可修改
- `--duration 0`：不自动停止，必须手动运行 `stop`。
- 默认录像格式：HD720、30 FPS、JPEG SVO2。

标签只能使用英文字母、数字、点、下划线和短横线，例如：

```text
red_ball_near
blue_ball_far
gate_front
negative_samples
turbid_water
```

启动成功后会显示保存目录，例如：

```text
Session: /home/auv/zed_datasets/20260927-103000-pool_ball_gate
SVO2: /home/auv/zed_datasets/20260927-103000-pool_ball_gate/raw.svo2
```

录像在后台运行，关闭 FinalShell 或断开 SSH 不会停止录像。

## 4. 查看状态和停止录像

查看状态、文件大小和剩余空间：

```bash
bash /home/auv/record_training_video.sh status
```

提前停止并安全保存：

```bash
bash /home/auv/record_training_video.sh stop
```

必须等待终端显示：

```text
Recorder stopped cleanly.
```

再关闭 Jetson 电源。直接断电可能导致 SVO2 没有正常封尾。

列出全部录像批次：

```bash
bash /home/auv/record_training_video.sh list
```

查看最新批次：

```bash
cat /home/auv/zed_datasets/.latest_session
```

## 5. 指定录像进行抽帧

抽帧前确认录像已经停止：

```bash
bash /home/auv/record_training_video.sh status
```

不要在录像为 `RUNNING` 时进行抽帧。

假设要处理的批次为：

```text
20260926-220904-pool_ball_gate
```

进入录像目录：

```bash
cd /home/auv/zed_datasets/20260926-220904-pool_ball_gate
```

每 10 帧抽取一张左目 JPG：

```bash
/home/auv/extract_svo_frames.py \
  ./raw.svo2 \
  --output ./yolo_frames_n10_q95 \
  --every-n 10 \
  --jpeg-quality 95
```

也可以明确指定系统 Python：

```bash
/usr/bin/python3 /home/auv/extract_svo_frames.py \
  ./raw.svo2 \
  --output ./yolo_frames_n10_q95 \
  --every-n 10 \
  --jpeg-quality 95
```

如果终端开头显示 `(base)`，不要使用下面这种写法：

```bash
python3 /home/auv/extract_svo_frames.py ...
```

因为此时 `python3` 指向 Miniforge Python 3.13，而 ZED SDK、OpenCV 和 NumPy 位于系统 Python 3.10。无需在 Miniforge 中重新安装 NumPy，直接使用脚本或 `/usr/bin/python3`。

抽帧间隔建议：

| 参数 | 30 FPS 下的密度 | 适用情况 |
|---|---:|---|
| `--every-n 5` | 每秒约 6 张 | 目标快速运动、短视频 |
| `--every-n 10` | 每秒约 3 张 | 默认推荐 |
| `--every-n 15` | 每秒约 2 张 | 普通慢速巡游 |
| `--every-n 30` | 每秒约 1 张 | 长视频、画面变化慢 |

不同间隔应使用不同输出目录，例如：

```text
yolo_frames_n5_q95
yolo_frames_n10_q95
yolo_frames_n30_q95
```

## 6. 检查抽帧结果

抽帧完成时会输出：

```text
OUTPUT=...
SOURCE_FRAMES=...
IMAGE_COUNT=...
EVERY_N=10
RESOLUTION=1280x720
```

检查完成标记：

```bash
test -s ./yolo_frames_n10_q95/.complete \
  && echo '抽帧完整' \
  || echo '抽帧未完成'
```

统计图片数量：

```bash
find ./yolo_frames_n10_q95 \
  -maxdepth 1 -type f -name 'frame_*.jpg' | wc -l
```

输出目录包含：

```text
yolo_frames_n10_q95/
├── frame_000000.jpg
├── frame_000010.jpg
├── frame_000020.jpg
├── ...
├── frames.csv
└── .complete
```

- `frame_*.jpg`：用于 YOLO 标注和训练的左目图片。
- `frames.csv`：图片名、SVO2 原始帧号和时间戳。
- `.complete`：完整抽取标志。

重复执行相同命令时，如果发现 `.complete`，程序会复用已有结果，不重复抽帧。

## 7. 使用 Windows FinalShell 下载图片

### 7.1 直接下载目录

1. 在 FinalShell 中连接 `192.168.137.1`。
2. 打开右侧或下方的 SFTP 文件管理器。
3. 进入：

```text
/home/auv/zed_datasets/20260926-220904-pool_ball_gate/
```

4. 找到：

```text
yolo_frames_n10_q95
```

5. 右键该目录，选择“下载”，保存到 Windows，例如：

```text
D:\AUV\dataset\20260926-220904-pool_ball_gate\
```

6. 下载后确认 JPG 数量与 Jetson 上 `wc -l` 的结果一致，并保留 `frames.csv`。

FinalShell 文件面板可能不显示以点开头的 `.complete`。可在终端用以下命令确认：

```bash
ls -la ./yolo_frames_n10_q95
```

### 7.2 图片较多时打包后下载（推荐）

大量小文件逐个通过 SFTP 下载比较慢。建议先在 Jetson 打成不压缩的 tar 包；JPEG 已经压缩，再使用 gzip 收益很小。

在当前录像目录执行：

```bash
tar -cf yolo_frames_n10_q95.tar yolo_frames_n10_q95
```

检查文件：

```bash
ls -lh yolo_frames_n10_q95.tar
```

然后在 FinalShell 文件管理器中下载：

```text
/home/auv/zed_datasets/20260926-220904-pool_ball_gate/yolo_frames_n10_q95.tar
```

Windows 11 PowerShell 解包：

```powershell
mkdir D:\AUV\dataset\20260926-220904-pool_ball_gate
tar -xf .\yolo_frames_n10_q95.tar `
  -C D:\AUV\dataset\20260926-220904-pool_ball_gate
```

也可以使用 7-Zip 解压。

## 8. 推荐的完整现场流程

```text
1. Jetson 上电并通过 FinalShell 登录
2. 检查/修正系统时间
3. 确认 Recorder: STOPPED
4. 启动定时录像
5. 完成水下场景采集
6. 等待自动停止，或手动执行 stop
7. 确认 Recorder stopped cleanly
8. 选择批次并抽取左目 JPG
9. 检查 .complete 和图片数量
10. 打包为 tar
11. 使用 FinalShell 下载到 Windows
12. 在 Windows 核对图片数量后再开始标注
```

## 9. 常见问题

### `ModuleNotFoundError: No module named 'numpy'`

原因：使用了 Miniforge `(base)` 中的 Python 3.13。

解决：

```bash
/usr/bin/python3 /home/auv/extract_svo_frames.py ...
```

### `input does not exist`

检查是否进入了正确的录像目录：

```bash
pwd
ls -lh raw.svo2
```

不要使用尚未赋值的 `${session}` 变量。进入录像目录后直接使用 `./raw.svo2` 最稳妥。

### `output exists without a completion marker`

说明之前抽帧异常中断，先保留现场并重命名：

```bash
mv yolo_frames_n10_q95 \
  "yolo_frames_n10_q95.incomplete-$(date +%Y%m%d-%H%M%S)"
```

然后重新执行抽帧命令。

### 磁盘空间不足

```bash
df -h /home/auv
du -h --max-depth=2 /home/auv/zed_datasets | sort -h | tail
```

确认 Windows 已完整下载并备份后，再决定是否清理 Jetson 数据；不要在未核对前删除原始 SVO2。
