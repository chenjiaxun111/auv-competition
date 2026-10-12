# 合并后的 YOLO 待标注图片集

## 数据概况

- 总图片数：2276
- 图片格式：JPG
- 分辨率：1280×720
- `test1`：942 张
- `test2`：654 张
- `test3`：680 张

## 目录结构

```text
merged_yolo_dataset/
├── images/                  # 合并后的待标注图片
├── source_manifests/        # 三次 SVO2 抽帧的原始清单和完成记录
├── classes.json             # 类别编号映射
└── README.md
```

为防止三个批次的同名帧互相覆盖，合并后的文件名增加了批次前缀：

```text
test1_frame_000000.jpg
test2_frame_000000.jpg
test3_frame_000000.jpg
```

`source_manifests/test1_frames.csv` 中的 `frame_000000.jpg` 对应 `images/test1_frame_000000.jpg`，其他批次以此类推。

## 类别映射

```text
0: red_ball
1: gate
2: blue_ball
```

## 重要提示

当前目录只合并了图片，**还没有 YOLO `.txt` 标签**，不能直接开始训练。不要批量创建空标签，否则会把实际含有门框或球的图片错误当成负样本。

标注完成后，再按原始视频批次划分训练集和验证集，避免将同一段连续视频的相邻帧随机分到两个集合中。最终测试集应使用另外录制的独立视频。

## 使用内置标注工具

当前工作区的标注脚本位于：

```text
AUV-Connection-Available/label_yolo_dataset.py
```

脚本由旧 AUV 工作空间的人工标注工具适配而来，已改为支持本数据集的三个类别、断点续标和独立 `labels/` 目录。

在标注前先检查数据：

```bash
/usr/bin/python3 /home/cjx/cjx_project/auv比赛/AUV-Connection-Available/label_yolo_dataset.py \
  --check-only
```

启动图形标注界面：

```bash
cd /home/cjx/cjx_project/auv比赛/训练集/1/merged_yolo_dataset
./start_annotation.sh
```

操作键：

- 鼠标左键拖动：绘制待加入的边界框。
- `1`：选择蓝球 `blue_ball`。
- `2`：选择红球 `red_ball`。
- `3`：选择门框 `gate`。
- `S`：加入当前边界框，一张图可以加入多个目标。
- `Enter` 或 `D`：保存当前图片并进入下一张。
- `N`：人工确认当前图片无目标，保存空标签并进入下一张。
- `K`：对当前图片存疑，跳过且不生成标签，留待复审。
- `Z`：撤销当前待选框或最后一个已加入的框。
- `C`：清空当前图片中的框，需再按 `Enter` 才会保存。
- `A`：返回上一张。
- `Q`：退出；下次启动会从第一张未审核图片继续。

标签保存在：

```text
merged_yolo_dataset/labels/<与图片同名>.txt
```

空 `.txt` 代表人工确认的负样本；不存在 `.txt` 代表尚未标注，两者不能混淆。需要复查所有已标注图片时使用：

```bash
/usr/bin/python3 /home/cjx/cjx_project/auv比赛/AUV-Connection-Available/label_yolo_dataset.py \
  --include-reviewed
```
